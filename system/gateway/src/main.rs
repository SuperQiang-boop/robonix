// SPDX-License-Identifier: MulanPSL-2.0
// robonix-gateway — unified user-facing entry layer.
//
// Design
// ──────
// All input modalities ultimately produce the same `Task` and receive the
// same `PlannerEvent` stream. The Gateway gRPC facade is contract
// `robonix/system/gateway`, which exposes:
//
//   * SubmitTask(Task) → stream PlannerEvent
//       For text / API / pre-built tasks. Gateway normalises `Task.user_id`
//       (defaulting to `local:<os_user>` when empty) and forwards to Planner.
//
//   * StartVoiceSession(...) → stream VoiceEvent
//       For push-to-talk voice. Gateway drives mic → ASR → voiceprint →
//       Planner → optional TTS → speaker, wrapping every stage as a
//       VoiceEvent. See `voice.rs`.
//
// Gateway itself does NOT contain ASR / TTS / voiceprint logic. Each is an
// Src-registered system capability (robonix/service/speech/asr,
// robonix/service/speech/tts, robonix/service/voiceprint/identify) discovered
// and called via gRPC — same pattern as the VLM service in Planner.

mod access;
mod handsfree;
mod pb;
mod voice;

use access::{AccessControlConfig, AccessDecision};
use anyhow::{Context, Result, anyhow};
use clap::Parser;
use pb::contracts::{
    robonix_system_gateway_handsfree_events_server::{
        RobonixSystemGatewayHandsfreeEvents, RobonixSystemGatewayHandsfreeEventsServer,
    },
    robonix_system_gateway_handsfree_set_enabled_server::{
        RobonixSystemGatewayHandsfreeSetEnabled, RobonixSystemGatewayHandsfreeSetEnabledServer,
    },
    robonix_system_gateway_handsfree_status_server::{
        RobonixSystemGatewayHandsfreeStatus, RobonixSystemGatewayHandsfreeStatusServer,
    },
    robonix_system_gateway_submit_server::{
        RobonixSystemGatewaySubmit, RobonixSystemGatewaySubmitServer,
    },
    robonix_system_gateway_voice_server::{
        RobonixSystemGatewayVoice, RobonixSystemGatewayVoiceServer,
    },
    robonix_system_planner_client::RobonixSystemPlannerClient,
};
use pb::gateway::{
    GetHandsfreeStatusRequest, GetHandsfreeStatusResponse, SetHandsfreeRequest,
    SetHandsfreeResponse, StartVoiceSessionRequest, VoiceEvent, WatchHandsfreeEventsRequest,
};
use pb::planner::rtdl_node_state::RtdlNodeStateEnum;
use pb::planner::{CapabilityCall, Plan, PlannerEvent, Task};
use robonix_log::{debug, info, warn};
use robonix_src::client::{self as src_client, SrcClient};
use robonix_src::pb as src_pb;
use std::pin::Pin;
use std::sync::Arc;
use std::time::Duration;
use tokio::sync::{Mutex, mpsc};
use tokio_stream::{Stream, StreamExt, wrappers::ReceiverStream};
use tonic::{Request, Response, Status};
use uuid::Uuid;

const GATEWAY_PROVIDER_ID: &str = "gateway";
const GATEWAY_NAMESPACE: &str = "robonix/system/gateway";
const GATEWAY_SUBMIT_CONTRACT: &str = "robonix/system/gateway/submit";
const GATEWAY_VOICE_CONTRACT: &str = "robonix/system/gateway/voice";
const GATEWAY_HANDSFREE_SET_CONTRACT: &str = "robonix/system/gateway/handsfree/set_enabled";
const GATEWAY_HANDSFREE_STATUS_CONTRACT: &str = "robonix/system/gateway/handsfree/status";
const GATEWAY_HANDSFREE_EVENTS_CONTRACT: &str = "robonix/system/gateway/handsfree/events";
const GATEWAY_SUBMIT_TOML: &str = "capabilities/system/gateway/submit.v1.toml";
const GATEWAY_VOICE_TOML: &str = "capabilities/system/gateway/voice.v1.toml";
const GATEWAY_HANDSFREE_SET_TOML: &str =
    "capabilities/system/gateway/handsfree/set_enabled.v1.toml";
const GATEWAY_HANDSFREE_STATUS_TOML: &str = "capabilities/system/gateway/handsfree/status.v1.toml";
const GATEWAY_HANDSFREE_EVENTS_TOML: &str = "capabilities/system/gateway/handsfree/events.v1.toml";

/// `lib/system/planner/msg/Task.msg` source: TEXT=0 AUDIO=1
const INTENT_SOURCE_TEXT: u32 = 0;
const INTENT_SOURCE_AUDIO: u32 = 1;

/// `lib/system/planner/msg/PlannerEvent.msg` event_kind.
const EVT_TEXT_CHUNK: u32 = 0;
const EVT_TASK_GRAPH: u32 = 1;
const EVT_BATCH_RESULT: u32 = 2;
const EVT_STATUS: u32 = 3;
const EVT_FINAL_TEXT: u32 = 4;

// ── GatewayPipeline ─────────────────────────────────────────────────────────
//
// Forward a `Task` to Planner (`RobonixSystemPlanner.SubmitTask`), return the event
// channel. Opens a new gRPC channel per call so gateway can start before
// Planner and survive Planner restarts without restarting itself. Planner endpoint
// is re-resolved through Src on every call (so a Planner restart on a new
// port is picked up automatically).

pub struct GatewayPipeline {
    planner_endpoint_default: String,
    src: Arc<Mutex<SrcClient>>,
    access: Arc<AccessControlConfig>,
}

impl GatewayPipeline {
    /// Create a Gateway pipeline with a shared access-control policy. Every
    /// text/API task is normalized first, then authorized before any Planner
    /// channel is opened.
    pub fn new(
        planner_endpoint_default: impl Into<String>,
        src: Arc<Mutex<SrcClient>>,
        access: Arc<AccessControlConfig>,
    ) -> Self {
        Self {
            planner_endpoint_default: planner_endpoint_default.into(),
            src,
            access,
        }
    }

    /// Forward `task` to Planner. Returns a channel that yields `PlannerEvent`s.
    pub async fn handle_intent(
        &self,
        mut task: Task,
    ) -> Result<mpsc::Receiver<Result<PlannerEvent, Status>>> {
        ensure_user_id(&mut task);
        let user_id = task_user_id(&task);
        match self.access.authorize_user(&user_id) {
            AccessDecision::Allow {
                user_id,
                method,
                reason,
                ..
            } => {
                info!("[gateway/access] text allow user={user_id} via {method:?}: {reason}");
            }
            AccessDecision::Deny {
                user_id, reason, ..
            } => {
                warn!("[gateway/access] text deny user={user_id}: {reason}");
                return Err(anyhow!("access denied for user '{user_id}': {reason}"));
            }
        }
        let (tx, rx) = mpsc::channel(64);

        let planner_ep = match resolve_planner_endpoint(&self.src).await {
            Some(ep) => ep,
            None => self.planner_endpoint_default.clone(),
        };

        let mut client = RobonixSystemPlannerClient::connect(planner_ep.clone())
            .await
            .with_context(|| format!("connect Planner at {planner_ep}"))?;
        let response = client
            .submit_task(Request::new(task))
            .await
            .with_context(|| format!("Planner SubmitTask at {planner_ep}"))?;
        let mut grpc = response.into_inner();
        tokio::spawn(async move {
            while let Some(item) = grpc.next().await {
                if tx
                    .send(item.map_err(|e| Status::internal(e.to_string())))
                    .await
                    .is_err()
                {
                    break;
                }
            }
        });
        Ok(rx)
    }
}

async fn resolve_planner_endpoint(src: &Arc<Mutex<SrcClient>>) -> Option<String> {
    let mut src = src.lock().await;
    let transport = src_pb::Transport::Grpc;
    let providers = src
        .query_capabilities("", "robonix/system/planner", transport)
        .await
        .ok()?;
    let provider = providers.iter().find(|r| {
        r.capabilities
            .iter()
            .any(|i| i.contract_id == "robonix/system/planner" && i.transport == transport as i32)
    })?;
    let (_channel_id, endpoint, _params) = src
        .connect_capability(
            GATEWAY_PROVIDER_ID,
            &provider.id,
            "robonix/system/planner",
            transport,
        )
        .await
        .ok()?;
    if endpoint.is_empty() {
        None
    } else {
        Some(localhost_to_ipv4(&endpoint))
    }
}

fn localhost_to_ipv4(ep: &str) -> String {
    let with_scheme = if ep.starts_with("http") {
        ep.to_string()
    } else {
        format!("http://{ep}")
    };
    with_scheme.replace("localhost", "127.0.0.1")
}

/// Default the user_id (stored inside `context_json.user_id` since Task itself
/// has no `user_id` field yet) to `local:<os_user>` and tag the modality so
/// Planner can tell text vs voice without inspecting `source`.
fn ensure_user_id(task: &mut Task) {
    let mut ctx: serde_json::Value = if task.context_json.trim().is_empty() {
        serde_json::json!({})
    } else {
        serde_json::from_str(&task.context_json).unwrap_or_else(|_| serde_json::json!({}))
    };
    if let Some(obj) = ctx.as_object_mut() {
        let has_user = obj
            .get("user_id")
            .and_then(|v| v.as_str())
            .is_some_and(|s| !s.is_empty());
        if !has_user {
            obj.insert(
                "user_id".to_string(),
                serde_json::json!(format!("local:{}", whoami::username())),
            );
        }
        let modality = match task.source {
            INTENT_SOURCE_TEXT => "text",
            INTENT_SOURCE_AUDIO => "audio",
            _ => "unknown",
        };
        obj.entry("modality").or_insert(serde_json::json!(modality));
    }
    task.context_json = ctx.to_string();
}

fn task_user_id(task: &Task) -> String {
    serde_json::from_str::<serde_json::Value>(&task.context_json)
        .ok()
        .and_then(|ctx| {
            ctx.get("user_id")
                .and_then(|v| v.as_str())
                .map(ToString::to_string)
        })
        .unwrap_or_default()
}

// ── SystemGateway gRPC impl ─────────────────────────────────────────────────

struct GatewayServiceImpl {
    pipeline: Arc<GatewayPipeline>,
    src: Arc<Mutex<SrcClient>>,
    planner_endpoint_default: String,
    access: Arc<AccessControlConfig>,
    handsfree: Arc<handsfree::HandsfreeController>,
}

#[tonic::async_trait]
impl RobonixSystemGatewaySubmit for GatewayServiceImpl {
    type SubmitTaskStream = ReceiverStream<Result<PlannerEvent, Status>>;

    async fn submit_task(
        &self,
        request: Request<Task>,
    ) -> Result<Response<Self::SubmitTaskStream>, Status> {
        let task = request.into_inner();
        let rx = self
            .pipeline
            .handle_intent(task)
            .await
            .map_err(|e| Status::unavailable(format!("Planner unreachable: {e:#}")))?;
        Ok(Response::new(ReceiverStream::new(rx)))
    }
}

#[tonic::async_trait]
impl RobonixSystemGatewayVoice for GatewayServiceImpl {
    type StartVoiceSessionStream =
        Pin<Box<dyn Stream<Item = Result<VoiceEvent, Status>> + Send + 'static>>;

    async fn start_voice_session(
        &self,
        request: Request<StartVoiceSessionRequest>,
    ) -> Result<Response<Self::StartVoiceSessionStream>, Status> {
        let req = request.into_inner();
        self.handsfree.suspend_capture().await;
        let stream = match voice::start_voice_session(
            req,
            Arc::clone(&self.src),
            self.planner_endpoint_default.clone(),
            Arc::clone(&self.access),
        )
        .await
        {
            Ok(stream) => stream,
            Err(status) => {
                self.handsfree.resume_capture().await;
                return Err(status);
            }
        };
        let (tx, rx) = mpsc::channel(64);
        let handsfree = Arc::clone(&self.handsfree);
        tokio::spawn(async move {
            let mut stream = Box::pin(stream);
            let mut capture_suspended = true;
            while let Some(item) = stream.next().await {
                if capture_suspended
                    && item
                        .as_ref()
                        .is_ok_and(|event| event.event_kind == voice::KIND_ASR_FINAL)
                {
                    handsfree.resume_capture().await;
                    capture_suspended = false;
                }
                if tx.send(item).await.is_err() {
                    break;
                }
            }
            if capture_suspended {
                handsfree.resume_capture().await;
            }
        });
        let boxed: Self::StartVoiceSessionStream = Box::pin(ReceiverStream::new(rx));
        Ok(Response::new(boxed))
    }
}

#[tonic::async_trait]
impl RobonixSystemGatewayHandsfreeSetEnabled for GatewayServiceImpl {
    async fn set_handsfree(
        &self,
        request: Request<SetHandsfreeRequest>,
    ) -> Result<Response<SetHandsfreeResponse>, Status> {
        let request = request.into_inner();
        let status = self
            .handsfree
            .set_enabled(
                request.enabled,
                request.mic_provider_id,
                request.speaker_provider_id,
            )
            .await
            .map_err(|error| Status::invalid_argument(error.to_string()))?;
        Ok(Response::new(SetHandsfreeResponse {
            ok: true,
            enabled: status.enabled,
            state: status.state,
            detail: String::new(),
        }))
    }
}

#[tonic::async_trait]
impl RobonixSystemGatewayHandsfreeStatus for GatewayServiceImpl {
    async fn get_handsfree_status(
        &self,
        _request: Request<GetHandsfreeStatusRequest>,
    ) -> Result<Response<GetHandsfreeStatusResponse>, Status> {
        Ok(Response::new(self.handsfree.snapshot().await))
    }
}

#[tonic::async_trait]
impl RobonixSystemGatewayHandsfreeEvents for GatewayServiceImpl {
    type WatchHandsfreeEventsStream = ReceiverStream<Result<VoiceEvent, Status>>;

    async fn watch_handsfree_events(
        &self,
        _request: Request<WatchHandsfreeEventsRequest>,
    ) -> Result<Response<Self::WatchHandsfreeEventsStream>, Status> {
        Ok(Response::new(self.handsfree.subscribe_events()))
    }
}

// ── Stdin text loop (headless fallback) ─────────────────────────────────────

async fn drain_session_end(pipeline: &GatewayPipeline, session_id: &str) {
    let task = Task {
        task_id: Uuid::new_v4().to_string(),
        session_id: session_id.to_string(),
        source: INTENT_SOURCE_TEXT,
        text: String::new(),
        audio_data: vec![],
        context_json: r#"{"session_end":true}"#.to_string(),
        timestamp_ms: now_ms(),
    };
    match pipeline.handle_intent(task).await {
        Ok(rx) => {
            let mut stream = ReceiverStream::new(rx);
            while stream.next().await.is_some() {}
        }
        Err(e) => debug!("[gateway/text] session_end: {e:#}"),
    }
}

async fn run_text_loop(pipeline: Arc<GatewayPipeline>) -> Result<()> {
    use std::io::{self, Write};

    let session_id = Uuid::new_v4().to_string();
    println!("[gateway/text] session {session_id}");
    println!("[gateway/text] type a message and press Enter  (Ctrl-D to exit)");

    let stdin = io::stdin();
    loop {
        print!("> ");
        io::stdout().flush()?;

        let mut line = String::new();
        if stdin.read_line(&mut line)? == 0 {
            drain_session_end(&pipeline, &session_id).await;
            break;
        }
        let text = line.trim().to_string();
        if text.is_empty() {
            continue;
        }

        let task = Task {
            task_id: Uuid::new_v4().to_string(),
            session_id: session_id.clone(),
            source: INTENT_SOURCE_TEXT,
            text,
            audio_data: vec![],
            context_json: String::new(),
            timestamp_ms: now_ms(),
        };

        match pipeline.handle_intent(task).await {
            Err(e) => eprintln!("[gateway/text] error: {e:#}"),
            Ok(rx) => {
                let mut stream = ReceiverStream::new(rx);
                let mut printing = false;
                while let Some(item) = stream.next().await {
                    let ev = match item {
                        Ok(e) => e,
                        Err(e) => {
                            eprintln!("[gateway/text] stream error: {e}");
                            break;
                        }
                    };
                    match ev.event_kind {
                        EVT_TEXT_CHUNK => {
                            if !printing {
                                print!("\nPlanner: ");
                                printing = true;
                            }
                            print!("{}", ev.text_chunk);
                            io::stdout().flush()?;
                        }
                        EVT_FINAL_TEXT => {
                            let t = ev.final_text.clone();
                            if printing {
                                println!();
                            } else {
                                println!("\nPlanner: {t}");
                            }
                            printing = false;
                        }
                        EVT_TASK_GRAPH => {
                            if let Some(ref p) = ev.plan {
                                if printing {
                                    println!();
                                    printing = false;
                                }
                                println!(
                                    "[round {}] dispatching {} call(s)…",
                                    p.round,
                                    plan_calls(p).len()
                                );
                                for c in plan_calls(p) {
                                    println!("  · {}", c.contract_id);
                                }
                            }
                        }
                        EVT_BATCH_RESULT => {
                            if let Some(ref r) = ev.batch_result {
                                let ok = r
                                    .results
                                    .iter()
                                    .filter(|x| x.state == RtdlNodeStateEnum::Succeeded as u32)
                                    .count();
                                println!(
                                    "[round {}] results: {ok} ok, {} failed",
                                    r.round,
                                    r.results.len() - ok
                                );
                            }
                        }
                        EVT_STATUS => {}
                        _ => {}
                    }
                }
                println!();
            }
        }
    }
    println!("\nBye.");
    Ok(())
}

// ── main ────────────────────────────────────────────────────────────────────

#[derive(Parser, Debug)]
#[command(
    name = "robonix-gateway",
    about = "Unified user-facing entry — text + voice → Planner"
)]
struct Args {
    /// gRPC listen address (host:port). Defaults to 127.0.0.1:50081 or
    /// $ROBONIX_GATEWAY_PORT (port-only).
    #[arg(long)]
    listen: Option<String>,

    /// Src endpoint. Defaults to $ROBONIX_SRC_ENDPOINT, $ROBONIX_SRC,
    /// then 127.0.0.1:50051.
    #[arg(long)]
    src: Option<String>,

    /// Planner fallback endpoint when Src can't yet resolve RobonixSystemPlanner.
    /// Defaults to $ROBONIX_PLANNER_ENDPOINT, then 127.0.0.1:50071.
    #[arg(long = "planner-endpoint")]
    planner_endpoint: Option<String>,

    /// Log level for this component (`debug`/`info`/`warn`/`error`). Sets the
    /// log-file level floor; falls back to `ROBONIX_LOG_FILE_LEVEL` / `info`.
    /// Normally arrives inside `--config-json`, not as a standalone flag.
    #[arg(long)]
    log: Option<String>,

    /// The component's `system.gateway` manifest block, serialized to JSON by
    /// rbnx and passed as one arg (`--config-json '{...}'`). Parsed by the binary
    /// itself; `robonix_log::init_from_config` reads the `log` key from it so
    /// the manifest's per-component level reaches the log.
    #[arg(long = "config-json")]
    config_json: Option<String>,
}

#[tokio::main]
async fn main() -> Result<()> {
    let args = Args::parse();
    // Apply the manifest's per-component `log:` level (delivered inside
    // --config-json) to log's file sink before the first log line.
    robonix_log::init_from_config("gateway", args.config_json.as_deref());
    info!("robonix-gateway starting");

    let src_endpoint = args
        .src
        .clone()
        .unwrap_or_else(|| env_first(&["ROBONIX_SRC_ENDPOINT", "ROBONIX_SRC"], "127.0.0.1:50051"));
    let src_http = if src_endpoint.starts_with("http") {
        src_endpoint.clone()
    } else {
        format!("http://{src_endpoint}")
    };

    let planner_endpoint = args
        .planner_endpoint
        .clone()
        .unwrap_or_else(|| env_first(&["ROBONIX_PLANNER_ENDPOINT"], "127.0.0.1:50071"));
    let planner_http = {
        let raw = if planner_endpoint.starts_with("http") {
            planner_endpoint.clone()
        } else {
            format!("http://{planner_endpoint}")
        };
        raw.replace("localhost", "127.0.0.1")
    };

    // --listen accepts host:port. If the manifest passes just a port (or
    // the user sets ROBONIX_GATEWAY_PORT), bind 0.0.0.0:<port>.
    let listen_addr: std::net::SocketAddr = if let Some(spec) = args.listen.clone() {
        if spec.contains(':') {
            spec.parse()
                .with_context(|| format!("invalid --listen '{spec}'"))?
        } else {
            format!("0.0.0.0:{spec}").parse()?
        }
    } else {
        let port: u16 = std::env::var("ROBONIX_GATEWAY_PORT")
            .ok()
            .and_then(|s| s.parse().ok())
            .unwrap_or(50081);
        format!("0.0.0.0:{port}").parse()?
    };
    let listen_port = listen_addr.port();
    let advertised = format!("127.0.0.1:{listen_port}");

    info!("connecting to src at {src_http}");
    let mut src = SrcClient::connect_with_retry(&src_http, 10, Duration::from_secs(2))
        .await
        .context("connect to src")?;

    src.register_service(GATEWAY_PROVIDER_ID, GATEWAY_NAMESPACE, "")
        .await
        .context("register gateway capability")?;
    src.declare_capability(
        GATEWAY_PROVIDER_ID,
        GATEWAY_SUBMIT_CONTRACT,
        src_pb::Transport::Grpc,
        &advertised,
        src_client::grpc_params(
            GATEWAY_SUBMIT_TOML,
            "robonix.contracts.RobonixSystemGatewaySubmit",
            "/robonix.contracts.RobonixSystemGatewaySubmit/SubmitTask",
        ),
    )
    .await
    .context("declare gateway submit gRPC capability")?;
    src.declare_capability(
        GATEWAY_PROVIDER_ID,
        GATEWAY_VOICE_CONTRACT,
        src_pb::Transport::Grpc,
        &advertised,
        src_client::grpc_params(
            GATEWAY_VOICE_TOML,
            "robonix.contracts.RobonixSystemGatewayVoice",
            "/robonix.contracts.RobonixSystemGatewayVoice/StartVoiceSession",
        ),
    )
    .await
    .context("declare gateway voice gRPC capability")?;
    src.declare_capability(
        GATEWAY_PROVIDER_ID,
        GATEWAY_HANDSFREE_SET_CONTRACT,
        src_pb::Transport::Grpc,
        &advertised,
        src_client::grpc_params(
            GATEWAY_HANDSFREE_SET_TOML,
            "robonix.contracts.RobonixSystemGatewayHandsfreeSetEnabled",
            "/robonix.contracts.RobonixSystemGatewayHandsfreeSetEnabled/SetHandsfree",
        ),
    )
    .await
    .context("declare gateway hands-free set gRPC capability")?;
    src.declare_capability(
        GATEWAY_PROVIDER_ID,
        GATEWAY_HANDSFREE_STATUS_CONTRACT,
        src_pb::Transport::Grpc,
        &advertised,
        src_client::grpc_params(
            GATEWAY_HANDSFREE_STATUS_TOML,
            "robonix.contracts.RobonixSystemGatewayHandsfreeStatus",
            "/robonix.contracts.RobonixSystemGatewayHandsfreeStatus/GetHandsfreeStatus",
        ),
    )
    .await
    .context("declare gateway hands-free status gRPC capability")?;
    src.declare_capability(
        GATEWAY_PROVIDER_ID,
        GATEWAY_HANDSFREE_EVENTS_CONTRACT,
        src_pb::Transport::Grpc,
        &advertised,
        src_client::grpc_params(
            GATEWAY_HANDSFREE_EVENTS_TOML,
            "robonix.contracts.RobonixSystemGatewayHandsfreeEvents",
            "/robonix.contracts.RobonixSystemGatewayHandsfreeEvents/WatchHandsfreeEvents",
        ),
    )
    .await
    .context("declare gateway hands-free events gRPC capability")?;
    // Gateway has no Driver(CMD_INIT/CMD_ACTIVATE) handshake — it's a Rust binary
    // that's fully ready as soon as the gRPC server is listening. Push the
    // state explicitly so `rbnx caps` shows ACTIVE instead of stopping at
    // the legacy-fallback INACTIVE that src infers from the first declare.
    if let Err(e) = src
        .set_lifecycle_state(GATEWAY_PROVIDER_ID, src_pb::LifecycleState::StateActive, "")
        .await
    {
        warn!("SetLifecycleState(ACTIVE) on {GATEWAY_PROVIDER_ID} failed: {e:#}");
    }
    info!("registered as '{GATEWAY_PROVIDER_ID}', SystemGateway gRPC on :{listen_port}");
    info!("robonix-gateway ready on :{listen_port}  (planner_default={planner_http})");

    {
        let mut hb = src.clone();
        tokio::spawn(async move {
            let mut tick = tokio::time::interval(Duration::from_secs(20));
            tick.tick().await;
            loop {
                tick.tick().await;
                if let Err(e) = hb.heartbeat(GATEWAY_PROVIDER_ID).await {
                    warn!("heartbeat failed: {e:#}");
                }
            }
        });
    }

    let src = Arc::new(Mutex::new(src));
    let access = Arc::new(AccessControlConfig::from_env());
    info!(
        "access gate enabled={} allowed_users={} voice_threshold={:.2}",
        access.enabled,
        access.allowed_users.len(),
        access.voice_threshold
    );
    let pipeline = Arc::new(GatewayPipeline::new(
        planner_http.clone(),
        Arc::clone(&src),
        Arc::clone(&access),
    ));
    let handsfree_config = args
        .config_json
        .as_deref()
        .map(serde_json::from_str::<handsfree::HandsfreeConfig>)
        .transpose()
        .context("parse gateway hands-free config")?
        .unwrap_or_default();
    let handsfree = handsfree::HandsfreeController::new(
        handsfree_config,
        Arc::clone(&src),
        planner_http.clone(),
        Arc::clone(&access),
    );
    handsfree.spawn();

    let source = std::env::var("ROBONIX_GATEWAY_SOURCE").unwrap_or_default();
    let text_handle: Option<tokio::task::JoinHandle<Result<()>>> = if source == "text" {
        info!("activating stdin text loop (headless mode)");
        Some(tokio::spawn(run_text_loop(Arc::clone(&pipeline))))
    } else {
        None
    };

    let svc = Arc::new(GatewayServiceImpl {
        pipeline,
        src: Arc::clone(&src),
        planner_endpoint_default: planner_http,
        access,
        handsfree,
    });
    let server = tonic::transport::Server::builder()
        .add_service(RobonixSystemGatewaySubmitServer::from_arc(Arc::clone(&svc)))
        .add_service(RobonixSystemGatewayVoiceServer::from_arc(Arc::clone(&svc)))
        .add_service(RobonixSystemGatewayHandsfreeSetEnabledServer::from_arc(
            Arc::clone(&svc),
        ))
        .add_service(RobonixSystemGatewayHandsfreeStatusServer::from_arc(
            Arc::clone(&svc),
        ))
        .add_service(RobonixSystemGatewayHandsfreeEventsServer::from_arc(svc))
        .serve(listen_addr);

    if let Some(handle) = text_handle {
        tokio::select! {
            res = server => { res?; }
            res = handle => { res??; }
        }
    } else {
        server.await?;
    }

    Ok(())
}

fn env_first(vars: &[&str], default: &str) -> String {
    for v in vars {
        if let Ok(val) = std::env::var(v)
            && !val.is_empty()
        {
            return val;
        }
    }
    default.to_string()
}

fn now_ms() -> u64 {
    std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .unwrap_or_default()
        .as_millis() as u64
}

fn plan_calls(plan: &Plan) -> Vec<&CapabilityCall> {
    plan.nodes
        .iter()
        .filter_map(|node| node.call.as_ref())
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn ensure_user_id_fills_default_for_text() {
        let mut t = Task {
            task_id: "t".into(),
            session_id: "s".into(),
            source: 0,
            text: "hi".into(),
            audio_data: vec![],
            context_json: String::new(),
            timestamp_ms: 0,
        };
        ensure_user_id(&mut t);
        let v: serde_json::Value = serde_json::from_str(&t.context_json).unwrap();
        let user_id = v["user_id"].as_str().unwrap();
        assert!(user_id.starts_with("local:"));
        assert_eq!(v["modality"], "text");
    }

    #[test]
    fn ensure_user_id_preserves_caller_value() {
        let mut t = Task {
            task_id: "t".into(),
            session_id: "s".into(),
            source: 1,
            text: "hi".into(),
            audio_data: vec![],
            context_json: r#"{"foo":"bar","user_id":"voice:alice"}"#.into(),
            timestamp_ms: 0,
        };
        ensure_user_id(&mut t);
        let v: serde_json::Value = serde_json::from_str(&t.context_json).unwrap();
        assert_eq!(v["modality"], "audio");
        assert_eq!(v["user_id"], "voice:alice");
        assert_eq!(v["foo"], "bar");
    }

    #[test]
    fn task_user_id_reads_context_field() {
        let t = Task {
            task_id: "t".into(),
            session_id: "s".into(),
            source: 0,
            text: "hi".into(),
            audio_data: vec![],
            context_json: r#"{"user_id":"local:alice"}"#.into(),
            timestamp_ms: 0,
        };
        assert_eq!(task_user_id(&t), "local:alice");
    }
}
