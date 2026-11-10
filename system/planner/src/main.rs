// SPDX-License-Identifier: MulanPSL-2.0
// Author: wheatfox <wheatfox17@icloud.com>
//
// robonix-planner — reasoning, planning, and session management.
//
// Run standalone (manual smoke testing):
//   robonix-planner --vlm-upstream https://api.openai.com/v1 \
//                 --vlm-api-key sk-... \
//                 --vlm-model gpt-5.5
//   # src defaults to 127.0.0.1:50051; everything else has sane defaults.
//
// Run under rbnx:
//   ROBONIX_SRC_ENDPOINT=… ROBONIX_CONFIG_PATH=/run/robonix/planner.yaml robonix-planner
//
// Either way, on startup planner:
//   1. Connects to src, registers its capability, declares the
//      `robonix/system/planner` gRPC capability.
//   2. Constructs an embedded LLM client from the resolved VLM config.
//   3. Serves RobonixSystemPlanner on `listen`. Executor address is discovered
//      through src at every Stream RPC, not configured statically.

mod config;
mod discovery;
mod history;
mod memory;
mod navigation_goal;
mod pb;
mod planner;
mod robotdesc_context;
mod service;
mod state_context;
mod task_control;
mod vlm;

use anyhow::{Context, Result};
use clap::Parser;
use config::{Args, PLANNER_NAMESPACE, PlannerConfig};
use pb::contracts::robonix_system_planner_get_health_server::RobonixSystemPlannerGetHealthServer;
use pb::contracts::robonix_system_planner_server::RobonixSystemPlannerServer;
use robonix_log::{info, warn};
use robonix_src::client::{self as src_client, SrcClient};
use robonix_src::pb as src_pb;
use service::PlannerServiceImpl;
use std::time::Duration;

#[tokio::main]
async fn main() -> Result<()> {
    let parsed = Args::parse();
    // Apply the manifest's per-component `log:` level (delivered inside
    // --config-json) to log's file sink before the first log line.
    robonix_log::init_from_config("planner", parsed.config_json.as_deref());
    info!("robonix-planner starting");

    let cfg = PlannerConfig::resolve(parsed)?;

    info!("connecting to src at {}", cfg.src_endpoint);
    let mut src = SrcClient::connect_with_retry(&cfg.src_endpoint, 10, Duration::from_secs(2))
        .await
        .context("connect to src")?;

    src.register_service(&cfg.id, PLANNER_NAMESPACE, "").await?;
    info!("registered as '{}' under '{PLANNER_NAMESPACE}'", cfg.id);

    let robotdesc_prompt_block =
        match robotdesc_context::fetch_system_prompt_block(&mut src, &cfg.id).await {
            Ok(Some(block)) => {
                info!("loaded Robotdesc body context into Planner system prompt");
                block
            }
            Ok(None) => String::new(),
            Err(e) => {
                warn!("Robotdesc body context load failed; continuing without it: {e:#}");
                String::new()
            }
        };

    let listen_addr: std::net::SocketAddr = cfg
        .listen
        .parse()
        .with_context(|| format!("invalid planner listen address '{}'", cfg.listen))?;
    // Use 127.0.0.1 (not "localhost") in the advertised endpoint so consumers
    // don't resolve ::1 while we listen on IPv4.
    let advertised = match listen_addr.ip() {
        std::net::IpAddr::V4(ip) if ip.is_unspecified() => {
            format!("127.0.0.1:{}", listen_addr.port())
        }
        _ => listen_addr.to_string(),
    };
    src.declare_capability(
        &cfg.id,
        "robonix/system/planner",
        src_pb::Transport::Grpc,
        &advertised,
        src_client::grpc_params(
            "capabilities/system/planner.v1.toml",
            "robonix.contracts.RobonixSystemPlanner",
            "/robonix.contracts.RobonixSystemPlanner/SubmitTask",
        ),
    )
    .await?;
    info!("declared RobonixSystemPlanner gRPC at {advertised}");

    src.declare_capability(
        &cfg.id,
        "robonix/system/planner/get_health",
        src_pb::Transport::Grpc,
        &advertised,
        src_client::grpc_params(
            "capabilities/system/planner/get_health.toml",
            "robonix.contracts.RobonixSystemPlannerGetHealth",
            "/robonix.contracts.RobonixSystemPlannerGetHealth/GetModuleHealth",
        ),
    )
    .await?;
    info!("declared RobonixSystemPlannerGetHealth gRPC at {advertised}");

    // Planner has no Driver lifecycle handshake — it's ready as soon as the
    // gRPC server is up. Push ACTIVE so `rbnx caps` doesn't show the
    // legacy-fallback INACTIVE forever.
    if let Err(e) = src
        .set_lifecycle_state(&cfg.id, src_pb::LifecycleState::StateActive, "")
        .await
    {
        warn!("SetLifecycleState(ACTIVE) failed: {e:#}");
    }

    let vlm = vlm::VlmClient::new(&cfg.vlm);
    info!(
        "VLM upstream='{}' model='{}'",
        cfg.vlm.upstream, cfg.vlm.model
    );

    // Src evicts providers after ~60s without a heartbeat. Send one every
    // 20s so we stay registered for the lifetime of the process.
    {
        let mut hb = src.clone();
        let provider_id = cfg.id.clone();
        tokio::spawn(async move {
            let mut tick = tokio::time::interval(Duration::from_secs(20));
            tick.tick().await; // first tick fires immediately; skip
            loop {
                tick.tick().await;
                if let Err(e) = hb.heartbeat(&provider_id).await {
                    warn!("heartbeat failed: {e:#}");
                }
            }
        });
    }

    let svc = PlannerServiceImpl::new(src, cfg.id.clone(), vlm, robotdesc_prompt_block);

    info!("RobonixSystemPlanner gRPC on {listen_addr}");
    info!("robonix-planner ready on {listen_addr}");

    tonic::transport::Server::builder()
        .add_service(RobonixSystemPlannerServer::new(svc.clone()))
        .add_service(RobonixSystemPlannerGetHealthServer::new(svc))
        .serve(listen_addr)
        .await
        .context("planner gRPC server failed")?;

    Ok(())
}
