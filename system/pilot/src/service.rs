// SPDX-License-Identifier: MulanPSL-2.0
// Author: wheatfox <wheatfox17@icloud.com>
//
// `RobonixSystemPilot` gRPC handler (contract `robonix/system/pilot`).

use crate::pb::contracts::{
    robonix_system_executor_control_plan_client::RobonixSystemExecutorControlPlanClient,
    robonix_system_executor_execute_client::RobonixSystemExecutorExecuteClient,
    robonix_system_executor_list_active_plans_client::RobonixSystemExecutorListActivePlansClient,
    robonix_system_pilot_get_health_server::RobonixSystemPilotGetHealth,
    robonix_system_pilot_server::RobonixSystemPilot,
};
use crate::pb::module_health::{
    GetModuleHealthRequest, GetModuleHealthResponse, ModuleHealth, ModuleHealthReport,
};
use crate::pb::pilot::{
    BatchResult, PilotEvent, Plan, RtdlNodeState, SessionStatusEvent, Task, TaskStateEvent,
};
use crate::planner::{self, ExecutorConn, TaskState};
use crate::vlm::{Message, VlmClient};
use anyhow::Context;
use robonix_atlas::client::{self as atlas_client, AtlasClient};
use robonix_scribe::{debug, error};
use std::collections::{HashMap, HashSet, VecDeque};
use std::sync::Arc;
use std::sync::atomic::{AtomicU64, Ordering};
use tokio::sync::{Mutex, broadcast, mpsc, watch};
use tokio_stream::wrappers::ReceiverStream;
use tonic::{Request, Response, Status};
use uuid::Uuid;

#[derive(Clone, Copy)]
#[repr(u32)]
#[allow(dead_code)]
pub enum SessionState {
    Active = 0,    // when pilot recieved a task
    Completed = 1, // the task is completed
    Failed = 2,    // the task failed
}

// `PilotEvent` carries one of N payloads tagged by `event_kind`. proto3 lacks
// a oneof here so we keep the discriminator explicit; planner + service both
// build events through `pack`.
pub const EVT_TEXT_CHUNK: u32 = 0;
pub const EVT_PLAN: u32 = 1;
pub const EVT_BATCH_RESULT: u32 = 2;
pub const EVT_STATUS: u32 = 3;
pub const EVT_FINAL_TEXT: u32 = 4;
pub const EVT_NODE_STATE: u32 = 5;
pub const EVT_TASK_STATE: u32 = 6;
const MODULE_HEALTH_SCHEMA_VERSION: u32 = 1;
const MODULE_HEALTH_OK: u32 = 0;
const MODULE_HEALTH_TTL_MS: u32 = 5000;

#[allow(dead_code)]
pub enum PilotStreamBody {
    TextChunk(String),
    FinalText(String),
    Plan(Plan),
    BatchResult(BatchResult),
    Status(SessionStatusEvent),
    NodeState(RtdlNodeState),
    TaskState(TaskStateEvent),
}

pub fn pack(session_id: &str, body: PilotStreamBody) -> PilotEvent {
    let mut e = PilotEvent {
        session_id: session_id.to_string(),
        ..Default::default()
    };
    match body {
        PilotStreamBody::TextChunk(s) => {
            e.event_kind = EVT_TEXT_CHUNK;
            e.text_chunk = s;
        }
        PilotStreamBody::Plan(g) => {
            e.event_kind = EVT_PLAN;
            e.plan = Some(g);
        }
        PilotStreamBody::BatchResult(b) => {
            e.event_kind = EVT_BATCH_RESULT;
            e.batch_result = Some(b);
        }
        PilotStreamBody::Status(s) => {
            e.event_kind = EVT_STATUS;
            e.status = Some(s);
        }
        PilotStreamBody::FinalText(s) => {
            e.event_kind = EVT_FINAL_TEXT;
            e.final_text = s;
        }
        PilotStreamBody::NodeState(ns) => {
            e.event_kind = EVT_NODE_STATE;
            e.node_state = Some(ns);
        }
        PilotStreamBody::TaskState(ts) => {
            e.event_kind = EVT_TASK_STATE;
            e.task_state = Some(ts);
        }
    }
    e
}

/// LLM conversation history per `session_id`. Grows across turns; never
/// expired (turns trim themselves at MAX_HISTORY in planner).
type Histories = Arc<Mutex<HashMap<String, Arc<Mutex<Vec<Message>>>>>>;
type TaskStates = Arc<Mutex<HashMap<String, Arc<Mutex<Option<TaskState>>>>>>;

type PendingPlans = Arc<std::sync::Mutex<HashSet<String>>>;

#[derive(Clone)]
struct ActiveTurnInput {
    turn_id: String,
    tx: mpsc::Sender<Task>,
    events: broadcast::Sender<Result<PilotEvent, String>>,
    reply_generation: Arc<AtomicU64>,
    pending_plans: PendingPlans,
}

/// Give each SubmitTask caller its own view of the active supervisor stream.
/// A final reply only closes the view once its observed plans have finished.
/// Subscribe and snapshot atomically with the relay, including same-session
/// steers that attach after a long-running plan was already dispatched.
fn subscribe_turn_events(
    events: &broadcast::Sender<Result<PilotEvent, String>>,
    reply_generation: &Arc<AtomicU64>,
    pending_plans: &PendingPlans,
) -> ReceiverStream<Result<PilotEvent, Status>> {
    view_turn_events(events, reply_generation, pending_plans, true)
}

/// Greeting voice observes the live turn without detaching the frontend's view.
fn view_turn_events(
    events: &broadcast::Sender<Result<PilotEvent, String>>,
    reply_generation: &Arc<AtomicU64>,
    pending_plans: &PendingPlans,
    take_reply: bool,
) -> ReceiverStream<Result<PilotEvent, Status>> {
    let generation = if take_reply {
        reply_generation.fetch_add(1, Ordering::AcqRel) + 1
    } else {
        reply_generation.load(Ordering::Acquire)
    };
    let reply_generation = Arc::clone(reply_generation);
    let (mut subscriber, mut pending) = {
        let plans = pending_plans.lock().expect("pending plan lock poisoned");
        (events.subscribe(), plans.clone())
    };
    let (tx, rx) = mpsc::channel(64);
    tokio::spawn(async move {
        loop {
            match subscriber.recv().await {
                Ok(Ok(mut event)) => {
                    // A newer same-session SubmitTask owns all subsequent
                    // user-facing events. Closing this stale view prevents one
                    // supervisor reply from being rendered by every historical
                    // request stream while leaving its RTDL trees untouched.
                    if reply_generation.load(Ordering::Acquire) != generation {
                        break;
                    }
                    match event.event_kind {
                        EVT_PLAN => {
                            if let Some(plan) = &event.plan {
                                pending.insert(plan.plan_id.clone());
                            }
                        }
                        EVT_BATCH_RESULT => {
                            if let Some(batch) = &event.batch_result {
                                pending.remove(&batch.plan_id);
                            }
                        }
                        EVT_FINAL_TEXT if !pending.is_empty() => {
                            // Acknowledging startup is not task completion. Keep
                            // forwarding real execution outcomes to this caller.
                            event = pack(
                                &event.session_id,
                                PilotStreamBody::Status(SessionStatusEvent {
                                    session_id: event.session_id.clone(),
                                    state: SessionState::Active as u32,
                                    message: format!(
                                        "{}（仍有 {} 个计划执行中，结束后将主动通知）",
                                        event.final_text,
                                        pending.len()
                                    ),
                                }),
                            );
                        }
                        _ => {}
                    }
                    let complete = event.event_kind == EVT_FINAL_TEXT;
                    if tx.send(Ok(event)).await.is_err() || complete {
                        break;
                    }
                }
                Ok(Err(error)) => {
                    let _ = tx.send(Err(Status::internal(error))).await;
                    break;
                }
                Err(broadcast::error::RecvError::Lagged(skipped)) => {
                    let _ = tx
                        .send(Err(Status::resource_exhausted(format!(
                            "Pilot event subscriber lagged by {skipped} event(s)"
                        ))))
                        .await;
                    break;
                }
                Err(broadcast::error::RecvError::Closed) => break,
            }
        }
    });
    ReceiverStream::new(rx)
}

/// Record execution lifetime and broadcast in the same critical section used
/// by subscription snapshots. Subscriber-local state then follows event order,
/// rather than racing against a plan that completed while its reply was queued.
fn relay_turn_event(
    events: &broadcast::Sender<Result<PilotEvent, String>>,
    pending_plans: &PendingPlans,
    item: Result<PilotEvent, String>,
) {
    let mut plans = pending_plans.lock().expect("pending plan lock poisoned");
    if let Ok(event) = &item {
        if let Some(plan) = &event.plan
            && event.event_kind == EVT_PLAN
        {
            plans.insert(plan.plan_id.clone());
        }
        if let Some(batch) = &event.batch_result
            && event.event_kind == EVT_BATCH_RESULT
        {
            plans.remove(&batch.plan_id);
        }
    }
    let _ = events.send(item);
}

const SEEN_TASK_IDS_PER_SESSION: usize = 256;

#[derive(Clone)]
pub struct PilotServiceImpl {
    /// `AtlasClient` is cheap to clone (its inner channel is just a handle);
    /// each Stream RPC clones it to discover executor concurrently without
    /// serialising on a single mutex.
    atlas: AtlasClient,
    /// Pilot's own provider_id; passed to atlas as `consumer_id` on every
    /// `ConnectCapability` so the channel record reflects who is using
    /// the executor.
    provider_id: String,
    vlm: VlmClient,
    soma_prompt_block: Arc<String>,
    histories: Histories,
    /// Harness-owned standing goal per session. It survives a transport turn
    /// that pauses for user input, so the next message cannot silently replace
    /// unfinished work with a model-authored summary.
    task_states: TaskStates,
    /// Per-session cancellation senders. `abort_turn` Task signals this
    /// without holding the history lock.
    cancels: Arc<Mutex<HashMap<String, watch::Sender<bool>>>>,
    /// Per-session steer queues. A Task submitted while a turn is already
    /// running for that session is pushed here as a mid-task steer instead of
    /// starting a second turn; the running `run_turn` drains it.
    steers: Arc<Mutex<HashMap<String, ActiveTurnInput>>>,
    /// Recently accepted task ids, scoped by session. A client retry with the
    /// same id is acknowledged exactly once and never starts or steers a turn
    /// twice. The bounded queue prevents an unbounded session-lifetime set.
    seen_task_ids: Arc<Mutex<HashMap<String, VecDeque<String>>>>,
    /// Process-global RTDL plan-id counter. Executor's active table is global,
    /// so ids must be unique across sessions as well as turns.
    plan_seq: Arc<AtomicU64>,
}

impl PilotServiceImpl {
    pub fn new(
        atlas: AtlasClient,
        provider_id: String,
        vlm: VlmClient,
        soma_prompt_block: String,
    ) -> Self {
        Self {
            atlas,
            provider_id,
            vlm,
            soma_prompt_block: Arc::new(soma_prompt_block),
            histories: Arc::new(Mutex::new(HashMap::new())),
            task_states: Arc::new(Mutex::new(HashMap::new())),
            cancels: Arc::new(Mutex::new(HashMap::new())),
            steers: Arc::new(Mutex::new(HashMap::new())),
            seen_task_ids: Arc::new(Mutex::new(HashMap::new())),
            plan_seq: Arc::new(AtomicU64::new(0)),
        }
    }

    async fn get_or_create_history(&self, session_id: &str) -> Arc<Mutex<Vec<Message>>> {
        let mut map = self.histories.lock().await;
        map.entry(session_id.to_string())
            .or_insert_with(|| Arc::new(Mutex::new(Vec::new())))
            .clone()
    }

    async fn get_or_create_task_state(&self, session_id: &str) -> Arc<Mutex<Option<TaskState>>> {
        let mut map = self.task_states.lock().await;
        map.entry(session_id.to_string())
            .or_insert_with(|| Arc::new(Mutex::new(None)))
            .clone()
    }

    async fn accept_task_id_once(&self, session_id: &str, task_id: &str) -> bool {
        if task_id.is_empty() {
            return true;
        }
        let mut sessions = self.seen_task_ids.lock().await;
        let ids = sessions.entry(session_id.to_string()).or_default();
        if ids.iter().any(|seen| seen == task_id) {
            return false;
        }
        ids.push_back(task_id.to_string());
        while ids.len() > SEEN_TASK_IDS_PER_SESSION {
            ids.pop_front();
        }
        true
    }
}

fn task_context(task: &Task) -> Option<serde_json::Value> {
    let raw = task.context_json.trim();
    if raw.is_empty() {
        return None;
    }
    serde_json::from_str(raw).ok()
}

fn task_is_abort_turn(task: &Task) -> bool {
    task_context(task)
        .and_then(|v| v.get("abort_turn").and_then(|x| x.as_bool()))
        .unwrap_or(false)
}

fn task_is_steer(task: &Task) -> bool {
    task_context(task).is_some_and(|v| {
        v.get("steer").and_then(|x| x.as_bool()).unwrap_or(false)
            || v.get("interaction_mode").and_then(|x| x.as_str()) == Some("steer")
    })
}

fn expected_turn_id(task: &Task) -> Option<String> {
    task_context(task).and_then(|v| {
        v.get("expected_turn_id")
            .and_then(|x| x.as_str())
            .filter(|id| !id.is_empty())
            .map(str::to_string)
    })
}

fn strict_expected_turn(task: &Task) -> bool {
    task_context(task)
        .and_then(|value| {
            value
                .get("strict_expected_turn")
                .and_then(|field| field.as_bool())
        })
        .unwrap_or(false)
}

#[tonic::async_trait]
impl RobonixSystemPilot for PilotServiceImpl {
    type SubmitTaskStream = ReceiverStream<Result<PilotEvent, Status>>;

    async fn submit_task(
        &self,
        request: Request<Task>,
    ) -> Result<Response<Self::SubmitTaskStream>, Status> {
        let mut task = request.into_inner();
        if !crate::task_control::greeting_task_allowed(&task) {
            return Err(Status::cancelled(
                "greeting voice request is stale or was stopped",
            ));
        }

        if task.session_id.is_empty() {
            task.session_id = Uuid::new_v4().to_string();
        }
        if task.task_id.is_empty() {
            task.task_id = Uuid::new_v4().to_string();
        }

        if !self
            .accept_task_id_once(&task.session_id, &task.task_id)
            .await
        {
            debug!(
                "[pilot] duplicate task ignored session={} task_id={}",
                task.session_id, task.task_id
            );
            let (_tx, rx) = tokio::sync::mpsc::channel::<Result<PilotEvent, Status>>(1);
            return Ok(Response::new(ReceiverStream::new(rx)));
        }

        if task_is_abort_turn(&task) {
            let id = task.session_id.clone();
            if let Err(error) = crate::task_control::stop_greeting(&id) {
                robonix_scribe::warn!("could not record greeting Stop for {id}: {error}");
            }
            let turn_signaled = if let Some(tx) = self.cancels.lock().await.get(&id) {
                tx.send_if_modified(|interrupted| {
                    if *interrupted {
                        false
                    } else {
                        *interrupted = true;
                        true
                    }
                })
            } else {
                false
            };
            debug!("[pilot] session-scoped stop session {id} (turn_signaled={turn_signaled})");
            let (tx, rx) = tokio::sync::mpsc::channel::<Result<PilotEvent, Status>>(1);
            let message = if turn_signaled {
                "stop requested for this session; its active RTDL plans are being cancelled"
            } else {
                "no active turn exists for this session"
            };
            let _ = tx
                .send(Ok(pack(
                    &id,
                    PilotStreamBody::Status(SessionStatusEvent {
                        session_id: id.clone(),
                        state: SessionState::Completed as u32,
                        message: message.to_string(),
                    }),
                )))
                .await;
            return Ok(Response::new(ReceiverStream::new(rx)));
        }

        // Decide — under a single `steers` lock — whether this task is a mid-task
        // steer for an already-live turn or the start of a new turn. Doing the
        // check and the registration atomically prevents a check-then-insert race
        // where two near-simultaneous submits for one session both start a turn.
        let (steer_tx, steer_rx) = mpsc::channel::<Task>(32);
        let (candidate_events, _) = broadcast::channel(128);
        let candidate_reply_generation = Arc::new(AtomicU64::new(0));
        let candidate_pending_plans = PendingPlans::default();
        let explicit_steer = task_is_steer(&task);
        let expected_turn = expected_turn_id(&task);
        let existing_turn = {
            let mut steers = self.steers.lock().await;
            match steers.get(&task.session_id) {
                Some(existing) => Some(existing.clone()),
                None => {
                    if explicit_steer {
                        debug!(
                            "[pilot] steer for session {} has no active turn; starting a new turn",
                            task.session_id
                        );
                    }
                    steers.insert(
                        task.session_id.clone(),
                        ActiveTurnInput {
                            turn_id: task.task_id.clone(),
                            tx: steer_tx.clone(),
                            events: candidate_events.clone(),
                            reply_generation: Arc::clone(&candidate_reply_generation),
                            pending_plans: Arc::clone(&candidate_pending_plans),
                        },
                    );
                    None
                }
            }
        };
        if let Some(existing) = existing_turn {
            if let Some(expected) = expected_turn
                && expected != existing.turn_id
            {
                if strict_expected_turn(&task) {
                    return Err(Status::failed_precondition(format!(
                        "steer expected turn {expected}, but active turn is {}",
                        existing.turn_id
                    )));
                }
                debug!(
                    "[pilot] accepting same-session steer with stale expected turn {} (active={})",
                    expected, existing.turn_id
                );
            }
            // A turn is already live: every new same-session task is a steer of
            // that supervisor. Subscribe before queueing it so this caller sees
            // the response produced for its input instead of receiving an empty
            // stream and being forced to stop the background plan.
            let id = task.session_id.clone();
            let greeting_voice =
                task_context(&task).is_some_and(|context| context["greeting_event_id"].is_string());
            let rx = if greeting_voice {
                view_turn_events(
                    &existing.events,
                    &existing.reply_generation,
                    &existing.pending_plans,
                    false,
                )
            } else {
                subscribe_turn_events(
                    &existing.events,
                    &existing.reply_generation,
                    &existing.pending_plans,
                )
            };
            let ok = existing.tx.send(task).await.is_ok();
            debug!("[pilot] steer task for session {id} (queued={ok})");
            if !ok {
                return Err(Status::unavailable(
                    "active Pilot turn stopped before steer was queued",
                ));
            }
            return Ok(Response::new(rx));
        }

        let history_arc = self.get_or_create_history(&task.session_id).await;
        let task_state_arc = self.get_or_create_task_state(&task.session_id).await;
        let plan_seq = Arc::clone(&self.plan_seq);
        // what is tokio's tx and rx:
        // https://docs.rs/tokio/latest/tokio/sync/mpsc/struct.Sender.html
        // https://tokio.rs/tokio/tutorial/channels
        // MPSC: Multiple Producer Single Consumer
        let (tx, mut internal_rx) = tokio::sync::mpsc::channel::<Result<PilotEvent, Status>>(64);
        let rx = subscribe_turn_events(
            &candidate_events,
            &candidate_reply_generation,
            &candidate_pending_plans,
        );
        let relay_events = candidate_events.clone();
        let relay_pending = candidate_pending_plans.clone();
        tokio::spawn(async move {
            while let Some(item) = internal_rx.recv().await {
                let shared = item.map_err(|status| status.message().to_string());
                relay_turn_event(&relay_events, &relay_pending, shared);
            }
        });
        let atlas = self.atlas.clone();
        let provider_id = self.provider_id.clone();
        let vlm = self.vlm.clone();
        let soma_prompt_block = Arc::clone(&self.soma_prompt_block);
        let session_id = task.session_id.clone();
        let cancels = Arc::clone(&self.cancels);
        let steers = Arc::clone(&self.steers);

        let (cancel_tx, cancel_rx) = watch::channel(false);
        cancels.lock().await.insert(session_id.clone(), cancel_tx);
        // `steer_tx`/`steer_rx` were created above; the sender is already
        // registered in `self.steers` under the atomic check, and `steer_rx`
        // moves into the turn below to drain mid-task steers.

        tokio::spawn(async move {
            let _ = tx
                .send(Ok(pack(
                    &session_id,
                    PilotStreamBody::Status(SessionStatusEvent {
                        session_id: session_id.clone(),
                        state: SessionState::Active as u32,
                        message: format!("turn_id={}", task.task_id),
                    }),
                )))
                .await;

            let mut atlas_for_turn = atlas.clone();
            let mut executor = match build_executor_conn(atlas, &provider_id).await {
                Ok(e) => e,
                Err(e) => {
                    let _ = tx
                        .send(Err(Status::unavailable(format!(
                            "cannot reach Executor via atlas: {e:#}"
                        ))))
                        .await;
                    cancels.lock().await.remove(&session_id);
                    steers.lock().await.remove(&session_id);
                    return;
                }
            };

            let mut history = history_arc.lock().await;
            let mut standing_task = task_state_arc.lock().await;
            if let Err(e) = planner::run_turn(
                &task,
                &mut history,
                &mut standing_task,
                &vlm,
                &mut executor,
                &mut atlas_for_turn,
                &provider_id,
                &tx,
                cancel_rx,
                steer_rx,
                plan_seq,
                soma_prompt_block.as_str(),
            )
            .await
            {
                error!("[pilot] turn error for session '{session_id}': {e:#}");
                let _ = tx.send(Err(Status::internal(e.to_string()))).await;
            }

            cancels.lock().await.remove(&session_id);
            steers.lock().await.remove(&session_id);
        });

        Ok(Response::new(rx))
    }
}

#[tonic::async_trait]
impl RobonixSystemPilotGetHealth for PilotServiceImpl {
    async fn get_module_health(
        &self,
        _request: Request<GetModuleHealthRequest>,
    ) -> Result<Response<GetModuleHealthResponse>, Status> {
        Ok(Response::new(GetModuleHealthResponse {
            report: Some(pilot_health_report(&self.provider_id)),
        }))
    }
}

fn pilot_health_report(provider_id: &str) -> ModuleHealthReport {
    ModuleHealthReport {
        schema_version: MODULE_HEALTH_SCHEMA_VERSION,
        module: Some(ModuleHealth {
            module_key: String::new(),
            module_id: "pilot".to_string(),
            provider_id: provider_id.to_string(),
            health: MODULE_HEALTH_OK,
            state: "active".to_string(),
            reason_code: "OK".to_string(),
            detail: "pilot serving".to_string(),
            source: String::new(),
            received_ts_ns: 0,
            ttl_ms: MODULE_HEALTH_TTL_MS,
        }),
    }
}

/// Connect to Executor's business-plan and out-of-band control RPCs.
/// Capability discovery (what the LLM may call) remains Atlas-driven and does
/// not expose the control RPC as an RTDL capability.
async fn build_executor_conn(
    mut atlas: AtlasClient,
    consumer_id: &str,
) -> anyhow::Result<ExecutorConn> {
    let (_, executor_provider_id, exec_ch) = atlas_client::connect_to_capability(
        &mut atlas,
        consumer_id,
        "robonix/system/executor/execute",
    )
    .await
    .context("connect_to_capability robonix/system/executor/execute")?;
    let (_, control_provider_id, control_ch) = atlas_client::connect_to_capability(
        &mut atlas,
        consumer_id,
        "robonix/system/executor/control_plan",
    )
    .await
    .context("connect_to_capability robonix/system/executor/control_plan")?;
    let (_, active_provider_id, active_ch) = atlas_client::connect_to_capability(
        &mut atlas,
        consumer_id,
        "robonix/system/executor/list_active_plans",
    )
    .await
    .context("connect_to_capability robonix/system/executor/list_active_plans")?;
    if control_provider_id != executor_provider_id || active_provider_id != executor_provider_id {
        anyhow::bail!(
            "Executor execute/control/active capabilities resolved to different providers: {executor_provider_id} vs {control_provider_id} vs {active_provider_id}"
        );
    }
    Ok(ExecutorConn {
        graph: RobonixSystemExecutorExecuteClient::new(exec_ch),
        control: RobonixSystemExecutorControlPlanClient::new(control_ch),
        active: RobonixSystemExecutorListActivePlansClient::new(active_ch),
    })
}

#[cfg(test)]
mod tests {
    use super::{
        EVT_FINAL_TEXT, EVT_STATUS, MODULE_HEALTH_OK, MODULE_HEALTH_SCHEMA_VERSION,
        MODULE_HEALTH_TTL_MS, PendingPlans, PilotStreamBody, expected_turn_id, pack,
        pilot_health_report, relay_turn_event, strict_expected_turn, subscribe_turn_events,
        task_is_abort_turn, task_is_steer,
    };
    use crate::pb::pilot::{BatchResult, PilotEvent, Plan, RtdlNodeState, Task};
    use tokio_stream::StreamExt;

    fn task(ctx: &str) -> Task {
        Task {
            task_id: "t".into(),
            session_id: "s".into(),
            source: 0,
            text: String::new(),
            audio_data: Vec::new(),
            context_json: ctx.into(),
            timestamp_ms: 0,
        }
    }

    #[test]
    fn pilot_health_report_uses_minimal_module_health_v1_fields() {
        let report = pilot_health_report("pilot");
        assert_eq!(report.schema_version, MODULE_HEALTH_SCHEMA_VERSION);

        let module = report.module.expect("module health");
        assert_eq!(module.module_id, "pilot");
        assert_eq!(module.provider_id, "pilot");
        assert_eq!(module.health, MODULE_HEALTH_OK);
        assert_eq!(module.state, "active");
        assert_eq!(module.reason_code, "OK");
        assert_eq!(module.detail, "pilot serving");
        assert_eq!(module.ttl_ms, MODULE_HEALTH_TTL_MS);

        assert!(module.module_key.is_empty());
        assert!(module.source.is_empty());
        assert_eq!(module.received_ts_ns, 0);
    }

    #[test]
    fn abort_turn_detected() {
        assert!(task_is_abort_turn(&task(r#"{"abort_turn":true}"#)));
        assert!(!task_is_abort_turn(&task(r#"{"abort_turn":false}"#)));
        assert!(!task_is_abort_turn(&task(r#"{"foo":1}"#)));
        assert!(!task_is_abort_turn(&task("")));
        assert!(!task_is_abort_turn(&task("not json")));
    }

    #[test]
    fn explicit_steer_and_expected_turn_are_parsed() {
        let value = task(r#"{"interaction_mode":"steer","expected_turn_id":"turn-7"}"#);
        assert!(task_is_steer(&value));
        assert_eq!(expected_turn_id(&value).as_deref(), Some("turn-7"));
        assert!(!strict_expected_turn(&value));
        assert!(strict_expected_turn(&task(
            r#"{"expected_turn_id":"turn-7","strict_expected_turn":true}"#
        )));
        assert!(task_is_steer(&task(r#"{"steer":true}"#)));
        assert!(!task_is_steer(&task(r#"{"interaction_mode":"task"}"#)));
    }

    #[tokio::test]
    async fn submit_subscriber_closes_at_its_final_text_boundary() {
        let (events, _) = tokio::sync::broadcast::channel(8);
        let generation = std::sync::Arc::new(std::sync::atomic::AtomicU64::new(0));
        let mut stream = subscribe_turn_events(&events, &generation, &Default::default());
        events
            .send(Ok(PilotEvent {
                event_kind: EVT_STATUS,
                ..Default::default()
            }))
            .unwrap();
        events
            .send(Ok(PilotEvent {
                event_kind: EVT_FINAL_TEXT,
                final_text: "still running".into(),
                ..Default::default()
            }))
            .unwrap();
        events
            .send(Ok(PilotEvent {
                event_kind: EVT_STATUS,
                ..Default::default()
            }))
            .unwrap();

        assert_eq!(stream.next().await.unwrap().unwrap().event_kind, EVT_STATUS);
        let final_event = stream.next().await.unwrap().unwrap();
        assert_eq!(final_event.event_kind, EVT_FINAL_TEXT);
        assert_eq!(final_event.final_text, "still running");
        assert!(stream.next().await.is_none());
    }

    /// Autonomous greeting input must not hide ongoing work from its frontend.
    #[tokio::test]
    async fn greeting_voice_keeps_frontend_status_stream_alive() {
        let (events, _) = tokio::sync::broadcast::channel(8);
        let generation = std::sync::Arc::new(std::sync::atomic::AtomicU64::new(0));
        let mut frontend = subscribe_turn_events(&events, &generation);
        let mut voice = super::view_turn_events(&events, &generation, false);
        events
            .send(Ok(PilotEvent {
                event_kind: EVT_FINAL_TEXT,
                final_text: "finished".into(),
                ..Default::default()
            }))
            .unwrap();
        assert_eq!(
            frontend.next().await.unwrap().unwrap().final_text,
            "finished"
        );
        assert_eq!(voice.next().await.unwrap().unwrap().final_text, "finished");
    }

    #[tokio::test]
    async fn newest_submit_subscriber_exclusively_owns_future_replies() {
        let (events, _) = tokio::sync::broadcast::channel(8);
        let generation = std::sync::Arc::new(std::sync::atomic::AtomicU64::new(0));
        let mut stale = subscribe_turn_events(&events, &generation, &Default::default());
        let mut current = subscribe_turn_events(&events, &generation, &Default::default());

        events
            .send(Ok(PilotEvent {
                event_kind: EVT_FINAL_TEXT,
                final_text: "one reply".into(),
                ..Default::default()
            }))
            .unwrap();

        assert!(stale.next().await.is_none());
        let final_event = current.next().await.unwrap().unwrap();
        assert_eq!(final_event.event_kind, EVT_FINAL_TEXT);
        assert_eq!(final_event.final_text, "one reply");
        assert!(current.next().await.is_none());
    }
    /// Send through the same relay as the live supervisor, preserving event order.
    fn publish(
        events: &tokio::sync::broadcast::Sender<Result<PilotEvent, String>>,
        pending: &PendingPlans,
        body: PilotStreamBody,
    ) {
        relay_turn_event(events, pending, Ok(pack("s", body)));
    }

    #[tokio::test]
    async fn startup_reply_does_not_hide_any_terminal_execution_outcome() {
        for (state, name) in [
            (2, "SUCCEEDED"),
            (3, "FAILED"),
            (4, "CANCELED"),
            (5, "TIMEOUT"),
        ] {
            let (events, _) = tokio::sync::broadcast::channel(32);
            let generation = Default::default();
            let pending = PendingPlans::default();
            let mut stream = subscribe_turn_events(&events, &generation, &pending);
            publish(
                &events,
                &pending,
                PilotStreamBody::Plan(Plan {
                    plan_id: "p".into(),
                    ..Default::default()
                }),
            );
            publish(
                &events,
                &pending,
                PilotStreamBody::FinalText("task started".into()),
            );
            let results = vec![RtdlNodeState {
                state,
                ..Default::default()
            }];
            let notice = crate::execution_notice::terminal(
                "s",
                "p",
                "explore",
                &results,
                state != 2,
                state == 4,
            );
            // Queue completion before the subscriber runs. Consulting only
            // shared current pending state would close on the old startup reply.
            publish(
                &events,
                &pending,
                PilotStreamBody::BatchResult(BatchResult {
                    plan_id: "p".into(),
                    results,
                    any_failed: state != 2,
                    ..Default::default()
                }),
            );
            publish(&events, &pending, PilotStreamBody::Status(notice.clone()));
            publish(
                &events,
                &pending,
                PilotStreamBody::FinalText(notice.message.clone()),
            );
            let mut received = Vec::new();
            while let Some(event) =
                tokio::time::timeout(std::time::Duration::from_secs(1), stream.next())
                    .await
                    .unwrap()
            {
                received.push(event.unwrap());
            }
            assert_eq!(received.len(), 5);
            assert_eq!(received[1].event_kind, EVT_STATUS);
            assert!(
                received
                    .iter()
                    .any(|e| e.status.as_ref().is_some_and(|s| s.message.contains(name)))
            );
            assert_eq!(received.last().unwrap().final_text, notice.message);
        }
    }

    #[tokio::test]
    async fn new_same_session_view_keeps_old_background_plan_until_it_finishes() {
        let (events, _) = tokio::sync::broadcast::channel(32);
        let generation = Default::default();
        let pending = PendingPlans::default();
        let mut old = subscribe_turn_events(&events, &generation, &pending);
        publish(
            &events,
            &pending,
            PilotStreamBody::Plan(Plan {
                plan_id: "old-plan".into(),
                ..Default::default()
            }),
        );
        let mut current = subscribe_turn_events(&events, &generation, &pending);
        publish(
            &events,
            &pending,
            PilotStreamBody::FinalText("cancellation requested, not confirmed".into()),
        );
        assert!(old.next().await.is_none());
        assert_eq!(
            current.next().await.unwrap().unwrap().event_kind,
            EVT_STATUS
        );
        publish(
            &events,
            &pending,
            PilotStreamBody::BatchResult(BatchResult {
                plan_id: "old-plan".into(),
                ..Default::default()
            }),
        );
        publish(
            &events,
            &pending,
            PilotStreamBody::FinalText("cancel confirmed".into()),
        );
        assert_eq!(
            current.next().await.unwrap().unwrap().event_kind,
            super::EVT_BATCH_RESULT
        );
        assert_eq!(
            current.next().await.unwrap().unwrap().final_text,
            "cancel confirmed"
        );
        assert!(current.next().await.is_none());
    }

    #[tokio::test]
    async fn one_finished_plan_does_not_close_an_unrelated_running_plan() {
        let (events, _) = tokio::sync::broadcast::channel(32);
        let generation = Default::default();
        let pending = PendingPlans::default();
        let mut stream = subscribe_turn_events(&events, &generation, &pending);
        for id in ["a", "b"] {
            publish(
                &events,
                &pending,
                PilotStreamBody::Plan(Plan {
                    plan_id: id.into(),
                    ..Default::default()
                }),
            );
        }
        publish(
            &events,
            &pending,
            PilotStreamBody::BatchResult(BatchResult {
                plan_id: "a".into(),
                ..Default::default()
            }),
        );
        publish(
            &events,
            &pending,
            PilotStreamBody::FinalText("a finished, b running".into()),
        );
        publish(
            &events,
            &pending,
            PilotStreamBody::BatchResult(BatchResult {
                plan_id: "b".into(),
                ..Default::default()
            }),
        );
        publish(
            &events,
            &pending,
            PilotStreamBody::FinalText("all finished".into()),
        );
        let mut received = Vec::new();
        while let Some(event) = stream.next().await {
            received.push(event.unwrap());
        }
        assert_eq!(received.len(), 6);
        assert_eq!(received[3].event_kind, EVT_STATUS);
        assert_eq!(received[5].final_text, "all finished");
    }
    #[derive(Clone)]
    struct SubscriptionFixture {
        events: tokio::sync::broadcast::Sender<Result<PilotEvent, String>>,
        pending: PendingPlans,
        generation: std::sync::Arc<std::sync::atomic::AtomicU64>,
        submits: std::sync::Arc<std::sync::atomic::AtomicUsize>,
    }

    #[tonic::async_trait]
    impl crate::pb::contracts::robonix_system_pilot_server::RobonixSystemPilot for SubscriptionFixture {
        type SubmitTaskStream =
            tokio_stream::wrappers::ReceiverStream<Result<PilotEvent, tonic::Status>>;

        /// Use the real production subscription/relay over a loopback gRPC connection.
        async fn submit_task(
            &self,
            _request: tonic::Request<Task>,
        ) -> Result<tonic::Response<Self::SubmitTaskStream>, tonic::Status> {
            self.submits
                .fetch_add(1, std::sync::atomic::Ordering::SeqCst);
            let stream = subscribe_turn_events(&self.events, &self.generation, &self.pending);
            publish(
                &self.events,
                &self.pending,
                PilotStreamBody::Plan(Plan {
                    plan_id: "p".into(),
                    ..Default::default()
                }),
            );
            publish(
                &self.events,
                &self.pending,
                PilotStreamBody::FinalText("started, still running".into()),
            );
            Ok(tonic::Response::new(stream))
        }
    }

    #[tokio::test]
    async fn grpc_subscription_delivers_every_terminal_state_without_another_submit() {
        use crate::pb::contracts::robonix_system_pilot_client::RobonixSystemPilotClient;
        use crate::pb::contracts::robonix_system_pilot_server::RobonixSystemPilotServer;
        use std::sync::{
            Arc,
            atomic::{AtomicUsize, Ordering},
        };
        for (state, label) in [
            (2, "SUCCEEDED"),
            (3, "FAILED"),
            (4, "CANCELED"),
            (5, "TIMEOUT"),
        ] {
            let (events, _) = tokio::sync::broadcast::channel(32);
            let fixture = SubscriptionFixture {
                events: events.clone(),
                pending: Default::default(),
                generation: Default::default(),
                submits: Arc::new(AtomicUsize::new(0)),
            };
            let listener = tokio::net::TcpListener::bind("127.0.0.1:0").await.unwrap();
            let endpoint = format!("http://{}", listener.local_addr().unwrap());
            let incoming = futures_util::stream::unfold(listener, |listener| async {
                let result = listener.accept().await.map(|(socket, _)| socket);
                Some((result, listener))
            });
            let (stop, stopped) = tokio::sync::oneshot::channel::<()>();
            let server = tokio::spawn(
                tonic::transport::Server::builder()
                    .add_service(RobonixSystemPilotServer::new(fixture.clone()))
                    .serve_with_incoming_shutdown(incoming, async {
                        let _ = stopped.await;
                    }),
            );
            let mut client = RobonixSystemPilotClient::connect(endpoint).await.unwrap();
            let mut stream = client.submit_task(task("{}")).await.unwrap().into_inner();
            assert_eq!(
                stream.message().await.unwrap().unwrap().event_kind,
                super::EVT_PLAN
            );
            // The starter reply must be progress, not an EOF-producing FinalText.
            assert_eq!(
                stream.message().await.unwrap().unwrap().event_kind,
                EVT_STATUS
            );
            let results = vec![RtdlNodeState {
                state,
                ..Default::default()
            }];
            let notice = crate::execution_notice::terminal(
                "s",
                "p",
                "explore",
                &results,
                state != 2,
                state == 4,
            );
            publish(
                &events,
                &fixture.pending,
                PilotStreamBody::BatchResult(BatchResult {
                    plan_id: "p".into(),
                    results,
                    any_failed: state != 2,
                    ..Default::default()
                }),
            );
            publish(
                &events,
                &fixture.pending,
                PilotStreamBody::Status(notice.clone()),
            );
            publish(
                &events,
                &fixture.pending,
                PilotStreamBody::FinalText(notice.message),
            );
            let result = tokio::time::timeout(std::time::Duration::from_secs(2), async {
                let mut notices = 0;
                while let Some(event) = stream.message().await.unwrap() {
                    if event.event_kind == EVT_STATUS
                        && event.status.is_some_and(|s| s.message.contains(label))
                    {
                        notices += 1;
                    }
                }
                notices
            })
            .await
            .unwrap();
            assert_eq!(result, 1);
            assert_eq!(fixture.submits.load(Ordering::SeqCst), 1);
            let _ = stop.send(());
            server.await.unwrap().unwrap();
        }
    }
}
