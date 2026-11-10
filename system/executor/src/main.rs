// SPDX-License-Identifier: MulanPSL-2.0
// Author: wheatfox <wheatfox17@icloud.com>
//
// robonix-executor — capability-call dispatch runtime.
// On startup executor:
//   1. Connects to src, registers as `com.robonix.system.executor`.
//   2. Declares its gRPC Execute and CancelAllPlans capabilities.
//   3. Declares built-in capabilities under `robonix/system/executor/builtin/<op>`
//      so planner's src-driven discovery surfaces them to the LLM as plain
//      capabilities. Calls hitting these contracts short-circuit to in-process
//      handlers in `dispatch::builtin` — no MCP loopback.
//   4. Serves Execute on `listen`. Per-call dispatch resolves provider via
//      `ConnectCapability(provider_id, contract_id, MCP)` on src.

mod config;
mod dispatch;
mod pb;
mod plan_runtime;
mod rtdl_wire;
mod service;

use anyhow::{Context, Result};
use clap::Parser;
use config::{Args, EXECUTOR_NAMESPACE, ExecutorConfig};
use dispatch::builtin::BUILTINS;
use pb::contracts::robonix_system_executor_cancel_all_plans_server::RobonixSystemExecutorCancelAllPlansServer;
use pb::contracts::robonix_system_executor_control_plan_server::RobonixSystemExecutorControlPlanServer;
use pb::contracts::robonix_system_executor_execute_server::RobonixSystemExecutorExecuteServer;
use pb::contracts::robonix_system_executor_get_health_server::RobonixSystemExecutorGetHealthServer;
use pb::contracts::robonix_system_executor_list_active_plans_server::RobonixSystemExecutorListActivePlansServer;
use robonix_log::{info, warn};
use robonix_src::client::{self as src_client, SrcClient};
use robonix_src::pb as src_pb;
use service::ExecutorServiceImpl;
use std::time::Duration;

#[tokio::main]
async fn main() -> Result<()> {
    let parsed = Args::parse();
    // Apply the manifest's per-component `log:` level (delivered inside
    // --config-json) to log's file sink before the first log line.
    robonix_log::init_from_config("executor", parsed.config_json.as_deref());
    info!("robonix-executor starting");

    let cfg = ExecutorConfig::resolve(parsed)?;

    info!("connecting to src at {}", cfg.src_endpoint);
    let mut src = SrcClient::connect_with_retry(&cfg.src_endpoint, 10, Duration::from_secs(2))
        .await
        .context("connect to src")?;

    src.register_service(&cfg.id, EXECUTOR_NAMESPACE, "")
        .await?;
    info!("registered as '{}' under '{EXECUTOR_NAMESPACE}'", cfg.id);

    let listen_addr: std::net::SocketAddr = cfg
        .listen
        .parse()
        .with_context(|| format!("invalid executor listen address '{}'", cfg.listen))?;
    let advertised = match listen_addr.ip() {
        std::net::IpAddr::V4(ip) if ip.is_unspecified() => {
            format!("127.0.0.1:{}", listen_addr.port())
        }
        _ => listen_addr.to_string(),
    };

    // Execute RPC: planner → executor for plan dispatch.
    src.declare_capability(
        &cfg.id,
        "robonix/system/executor/execute",
        src_pb::Transport::Grpc,
        &advertised,
        src_client::grpc_params(
            "capabilities/system/executor/execute.v1.toml",
            "robonix.contracts.RobonixSystemExecutorExecute",
            "/robonix.contracts.RobonixSystemExecutorExecute/Execute",
        ),
    )
    .await?;

    // Out-of-band RTDL meta operations. These never enter PlanRuntime as a
    // new plan, so canceling work cannot create a self-referential cancel tree.
    src.declare_capability(
        &cfg.id,
        "robonix/system/executor/control_plan",
        src_pb::Transport::Grpc,
        &advertised,
        src_client::grpc_params(
            "capabilities/system/executor/control_plan.v1.toml",
            "robonix.contracts.RobonixSystemExecutorControlPlan",
            "/robonix.contracts.RobonixSystemExecutorControlPlan/ControlPlan",
        ),
    )
    .await?;

    // Read-only control path for clients and observability. Polling it must not
    // create an RTDL query plan of its own.
    src.declare_capability(
        &cfg.id,
        "robonix/system/executor/list_active_plans",
        src_pb::Transport::Grpc,
        &advertised,
        src_client::grpc_params(
            "capabilities/system/executor/list_active_plans.v1.toml",
            "robonix.contracts.RobonixSystemExecutorListActivePlans",
            "/robonix.contracts.RobonixSystemExecutorListActivePlans/ListActivePlans",
        ),
    )
    .await?;

    // CancelAllPlans RPC: control path for cancelling every active RTDL plan.
    src.declare_capability(
        &cfg.id,
        "robonix/system/executor/cancel_all_plans",
        src_pb::Transport::Grpc,
        &advertised,
        src_client::grpc_params(
            "capabilities/system/executor/cancel_all_plans.v1.toml",
            "robonix.contracts.RobonixSystemExecutorCancelAllPlans",
            "/robonix.contracts.RobonixSystemExecutorCancelAllPlans/CancelAll",
        ),
    )
    .await?;

    // Module health RPC: Monitor polls this for system-module health.
    src.declare_capability(
        &cfg.id,
        "robonix/system/executor/get_health",
        src_pb::Transport::Grpc,
        &advertised,
        src_client::grpc_params(
            "capabilities/system/executor/get_health.toml",
            "robonix.contracts.RobonixSystemExecutorGetHealth",
            "/robonix.contracts.RobonixSystemExecutorGetHealth/GetModuleHealth",
        ),
    )
    .await?;

    // Built-in capabilities: declared as MCP-transport capabilities so planner's
    // catalog discovery sees them like any user MCP provider. The endpoint is a
    // sentinel — dispatch never dials it; calls hitting these contracts hit
    // the provider_id == self short-circuit in `dispatch::dispatch`.
    let builtin_endpoint = format!("internal://{}/builtin", cfg.id);
    for spec in BUILTINS {
        let contract_id = format!("{EXECUTOR_NAMESPACE}/builtin/{}", spec.op);
        src.declare_capability_with_description(
            &cfg.id,
            &contract_id,
            src_pb::Transport::Mcp,
            &builtin_endpoint,
            src_client::mcp_params(spec.input_schema_json),
            spec.description,
        )
        .await
        .with_context(|| format!("declare builtin '{}'", contract_id))?;
    }
    info!(
        "declared executor gRPC capabilities + {} builtin capabilities at {advertised}",
        BUILTINS.len()
    );

    // Executor has no Driver lifecycle handshake — it's ready as soon as
    // the gRPC server is up. Push ACTIVE so `rbnx caps` doesn't show the
    // legacy-fallback INACTIVE forever.
    if let Err(e) = src
        .set_lifecycle_state(&cfg.id, src_pb::LifecycleState::StateActive, "")
        .await
    {
        warn!("SetLifecycleState(ACTIVE) failed: {e:#}");
    }

    // Src evicts providers after ~60s without a heartbeat. Send one every
    // 20s so we stay registered for the lifetime of the process.
    {
        let mut hb = src.clone();
        let provider_id = cfg.id.clone();
        tokio::spawn(async move {
            let mut tick = tokio::time::interval(Duration::from_secs(20));
            tick.tick().await;
            loop {
                tick.tick().await;
                if let Err(e) = hb.heartbeat(&provider_id).await {
                    warn!("heartbeat failed: {e:#}");
                }
            }
        });
    }

    let svc = ExecutorServiceImpl::new(src, cfg.id.clone());
    info!("executor gRPC on {listen_addr}");
    info!("robonix-executor ready on {listen_addr}");

    tonic::transport::Server::builder()
        .add_service(RobonixSystemExecutorExecuteServer::new(svc.clone()))
        .add_service(RobonixSystemExecutorCancelAllPlansServer::new(svc.clone()))
        .add_service(RobonixSystemExecutorControlPlanServer::new(svc.clone()))
        .add_service(RobonixSystemExecutorListActivePlansServer::new(svc.clone()))
        .add_service(RobonixSystemExecutorGetHealthServer::new(svc))
        .serve(listen_addr)
        .await
        .context("executor gRPC server failed")?;

    Ok(())
}
