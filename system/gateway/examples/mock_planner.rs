// SPDX-License-Identifier: MulanPSL-2.0
// examples/mock_planner.rs — minimal RobonixSystemPlanner stand-in for the voice demo.
//
// Replaces robonix-planner for tests that only need to verify the
// Gateway ↔ Planner wiring (no VLM, no Executor, no skill index).
//
// Behaviour:
//   * Registers as `com.robonix.demo.mock_planner` in Src under
//     `robonix/system/planner` (same contract as the real Planner).
//   * Implements RobonixSystemPlanner.SubmitTask:
//       - On `{"abort_turn":true}` / `{"session_end":true}` → close immediately.
//       - Otherwise → emit one TEXT_CHUNK + one FINAL_TEXT echoing the
//         incoming Task (text + user_id), so the demo client can verify the
//         user identity is propagated end-to-end.

use anyhow::Result;
use robonix_gateway::pb::contracts::robonix_system_planner_server::{
    RobonixSystemPlanner, RobonixSystemPlannerServer,
};
use robonix_gateway::pb::planner::{PlannerEvent, Task};
use robonix_log::{info, warn};
use robonix_src::client::{self as src_client, SrcClient};
use robonix_src::pb as src_pb;
use std::time::Duration;
use tokio::sync::mpsc;
use tokio_stream::wrappers::ReceiverStream;
use tonic::{Request, Response, Status};

const CAPABILITY_ID: &str = "com.robonix.demo.mock_planner";
const NAMESPACE: &str = "robonix/system/planner";
const CONTRACT_ID: &str = "robonix/system/planner";

const EVT_TEXT_CHUNK: u32 = 0;
const EVT_FINAL_TEXT: u32 = 4;

fn extract_user_id(task: &Task) -> String {
    let v: serde_json::Value =
        serde_json::from_str(task.context_json.trim()).unwrap_or_else(|_| serde_json::json!({}));
    v.get("user_id")
        .and_then(|x| x.as_str())
        .unwrap_or("(none)")
        .to_string()
}

fn task_is_control(task: &Task) -> bool {
    let j = task.context_json.trim();
    if j.is_empty() {
        return false;
    }
    let v: serde_json::Value = match serde_json::from_str(j) {
        Ok(v) => v,
        Err(_) => return false,
    };
    v.get("abort_turn")
        .and_then(|x| x.as_bool())
        .unwrap_or(false)
        || v.get("session_end")
            .and_then(|x| x.as_bool())
            .unwrap_or(false)
}

#[derive(Default)]
struct MockPlanner;

#[tonic::async_trait]
impl RobonixSystemPlanner for MockPlanner {
    type SubmitTaskStream = ReceiverStream<Result<PlannerEvent, Status>>;

    async fn submit_task(
        &self,
        request: Request<Task>,
    ) -> Result<Response<Self::SubmitTaskStream>, Status> {
        let task = request.into_inner();
        let (tx, rx) = mpsc::channel::<Result<PlannerEvent, Status>>(8);

        if task_is_control(&task) {
            info!(
                "[mock-planner] control task ({}); closing stream",
                task.context_json
            );
            return Ok(Response::new(ReceiverStream::new(rx)));
        }

        let user_id = extract_user_id(&task);
        let echo = format!(
            "[mock-planner] received from user_id='{user_id}' source={} text='{}' ctx={}",
            task.source, task.text, task.context_json
        );
        info!("{echo}");

        tokio::spawn(async move {
            let chunk = PlannerEvent {
                event_kind: EVT_TEXT_CHUNK,
                session_id: task.session_id.clone(),
                text_chunk: format!("(mock) ack from {user_id} → "),
                plan: None,
                batch_result: None,
                status: None,
                final_text: String::new(),
                node_state: None,
                task_state: None,
            };
            let _ = tx.send(Ok(chunk)).await;
            tokio::time::sleep(Duration::from_millis(80)).await;
            let final_ev = PlannerEvent {
                event_kind: EVT_FINAL_TEXT,
                session_id: task.session_id.clone(),
                text_chunk: String::new(),
                plan: None,
                batch_result: None,
                status: None,
                final_text: format!("you said \"{}\"", task.text),
                node_state: None,
                task_state: None,
            };
            let _ = tx.send(Ok(final_ev)).await;
        });

        Ok(Response::new(ReceiverStream::new(rx)))
    }
}

#[tokio::main]
async fn main() -> Result<()> {
    robonix_log::init("mock_planner");
    info!("mock planner starting");

    let src = std::env::var("ROBONIX_SRC").unwrap_or_else(|_| "127.0.0.1:50051".to_string());
    let src_http = if src.starts_with("http") {
        src
    } else {
        format!("http://{src}")
    };

    let port: u16 = std::env::var("MOCK_PLANNER_PORT")
        .ok()
        .and_then(|s| s.parse().ok())
        .unwrap_or(50071);
    let listen: std::net::SocketAddr = format!("0.0.0.0:{port}").parse()?;
    let advertised = format!("127.0.0.1:{port}");

    info!("[mock-planner] connecting to Src at {src_http}");
    let mut src = SrcClient::connect_with_retry(&src_http, 10, Duration::from_secs(2)).await?;
    src.register_service(CAPABILITY_ID, NAMESPACE, "").await?;
    src.declare_capability(
        CAPABILITY_ID,
        CONTRACT_ID,
        src_pb::Transport::Grpc,
        &advertised,
        src_client::grpc_params(
            "capabilities/system/planner.v1.toml",
            "robonix.contracts.RobonixSystemPlanner",
            "/robonix.contracts.RobonixSystemPlanner/SubmitTask",
        ),
    )
    .await?;
    info!("[mock-planner] registered as '{CAPABILITY_ID}', listening on {advertised}");
    eprintln!("mock-planner ready on :{port}");

    {
        let mut hb = src.clone();
        tokio::spawn(async move {
            let mut tick = tokio::time::interval(Duration::from_secs(20));
            tick.tick().await;
            loop {
                tick.tick().await;
                if let Err(e) = hb.heartbeat(CAPABILITY_ID).await {
                    warn!("heartbeat failed: {e:#}");
                }
            }
        });
    }

    tonic::transport::Server::builder()
        .add_service(RobonixSystemPlannerServer::new(MockPlanner))
        .serve(listen)
        .await?;
    Ok(())
}
