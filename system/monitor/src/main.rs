// SPDX-License-Identifier: MulanPSL-2.0
//
// robonix-monitor — health monitoring: power state, component health, threshold alerts.
// On startup monitor:
//   1. Connects to src, registers as `monitor`.
//   2. Declares gRPC capabilities:
//      - robonix/system/monitor/get    (rpc: GetMonitor → MonitorSnapshot)
//      - robonix/system/monitor/stream (topic_out: StreamMonitor → stream MonitorSnapshot)
//      - robonix/system/monitor/modules/get (rpc: GetModuleHealthSnapshot)
//   3. Consumes Robotdesc's StreamHealth gRPC stream (real or mock), normalizes
//      RobotdescHealthSnapshot → MonitorSnapshot via threshold rules.
//   4. Serves all gRPC services on `listen`.
//
// Monitor requires Robotdesc.  Use --mock-robotdesc to run an embedded mock, or point
// --robotdesc-endpoint at a real Robotdesc instance.

mod config;
mod mock_robotdesc;
pub mod module_health;
mod module_health_poll;
mod pb;
mod robotdesc_ingest;
mod service;
mod subprocess;

use anyhow::{Context, Result};
use clap::Parser;
use config::{Args, MONITOR_NAMESPACE, MonitorConfig};
use log::info;
use pb::contracts::robonix_system_monitor_get_server::RobonixSystemMonitorGetServer;
use pb::contracts::robonix_system_monitor_modules_get_server::RobonixSystemMonitorModulesGetServer;
use pb::contracts::robonix_system_monitor_stream_server::RobonixSystemMonitorStreamServer;
use robonix_src::client::{self as src_client, SrcClient};
use robonix_src::pb as src_pb;
use service::MonitorServiceImpl;
use std::sync::OnceLock;
use std::time::Duration;
use std::time::Instant;

#[tokio::main]
async fn main() -> Result<()> {
    let parsed = Args::parse();
    let log_filter = parsed
        .log
        .clone()
        .or_else(|| std::env::var("RUST_LOG").ok())
        .unwrap_or_else(|| "robonix_monitor=info".to_string());
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or(log_filter)).init();

    let cfg = MonitorConfig::resolve(parsed)?;
    if cfg.mock_robotdesc {
        return mock_robotdesc::run_mock_robotdesc(
            &cfg.src_endpoint,
            &cfg.mock_robotdesc_id,
            &cfg.mock_robotdesc_listen,
            &cfg.mock_robotdesc_scenario,
            cfg.mock_robotdesc_interval_ms,
            cfg.mock_robotdesc_arm.clone(),
        )
        .await;
    }

    info!("connecting to src at {}", cfg.src_endpoint);
    let mut src = SrcClient::connect_with_retry(&cfg.src_endpoint, 10, Duration::from_secs(2))
        .await
        .context("connect to src")?;

    src.register_service(&cfg.id, MONITOR_NAMESPACE, "").await?;
    info!("registered as '{}' under '{MONITOR_NAMESPACE}'", cfg.id);

    let listen_addr: std::net::SocketAddr = cfg
        .listen
        .parse()
        .with_context(|| format!("invalid monitor listen address '{}'", cfg.listen))?;
    let advertised = match listen_addr.ip() {
        std::net::IpAddr::V4(ip) if ip.is_unspecified() => {
            format!("127.0.0.1:{}", listen_addr.port())
        }
        _ => listen_addr.to_string(),
    };

    // Declare GetMonitor RPC.
    src.declare_capability(
        &cfg.id,
        "robonix/system/monitor/get",
        src_pb::Transport::Grpc,
        &advertised,
        src_client::grpc_params(
            "capabilities/system/monitor/get.v1.toml",
            "robonix.contracts.RobonixSystemMonitorGet",
            "/robonix.contracts.RobonixSystemMonitorGet/GetMonitor",
        ),
    )
    .await?;
    info!("declared GetMonitor at {advertised}");

    // Declare StreamMonitor server_stream.
    src.declare_capability(
        &cfg.id,
        "robonix/system/monitor/stream",
        src_pb::Transport::Grpc,
        &advertised,
        src_client::grpc_params(
            "capabilities/system/monitor/stream.v1.toml",
            "robonix.contracts.RobonixSystemMonitorStream",
            "/robonix.contracts.RobonixSystemMonitorStream/StreamMonitor",
        ),
    )
    .await?;
    info!("declared StreamMonitor at {advertised}");

    // Declare module health aggregate snapshot RPC.
    src.declare_capability(
        &cfg.id,
        "robonix/system/monitor/modules/get",
        src_pb::Transport::Grpc,
        &advertised,
        src_client::grpc_params(
            "capabilities/system/monitor/modules/get.toml",
            "robonix.contracts.RobonixSystemMonitorModulesGet",
            "/robonix.contracts.RobonixSystemMonitorModulesGet/GetModuleHealthSnapshot",
        ),
    )
    .await?;
    info!("declared ModuleHealthSnapshot at {advertised}");

    if let Err(e) = src
        .set_lifecycle_state(&cfg.id, src_pb::LifecycleState::StateActive, "")
        .await
    {
        log::warn!("SetLifecycleState(ACTIVE) failed: {e:#}");
    }

    // Heartbeat every 20s to prevent Src eviction (90s timeout).
    {
        let mut hb = src.clone();
        let provider_id = cfg.id.clone();
        tokio::spawn(async move {
            let mut tick = tokio::time::interval(Duration::from_secs(20));
            tick.tick().await;
            loop {
                tick.tick().await;
                if let Err(e) = hb.heartbeat(&provider_id).await {
                    log::warn!("heartbeat failed: {e:#}");
                }
            }
        });
    }

    // Build the shared service state.
    let svc = MonitorServiceImpl::new();
    svc.update_self_module_health(&cfg.id)
        .await
        .context("initialize Monitor self module health")?;
    svc.apply_expected_module_config(&cfg.expected_modules)
        .await;

    module_health_poll::spawn_module_health_poller(
        src.clone(),
        cfg.id.clone(),
        svc.clone(),
        cfg.expected_modules.clone(),
    );

    let robotdesc_rules: Vec<robotdesc_ingest::RobotdescThresholdRule> =
        match std::fs::read_to_string(&cfg.thresholds_path) {
            Ok(yaml_str) => match robotdesc_ingest::load_robotdesc_thresholds(&yaml_str) {
                Ok(r) => {
                    info!(
                        "loaded {} Robotdesc threshold rules from {}",
                        r.len(),
                        cfg.thresholds_path.display()
                    );
                    r
                }
                Err(e) => {
                    log::error!(
                        "failed to parse Robotdesc threshold file '{}': {e:#}; using defaults",
                        cfg.thresholds_path.display()
                    );
                    robotdesc_ingest::default_thresholds()
                }
            },
            Err(_) => robotdesc_ingest::default_thresholds(),
        };

    // ── ROBOTDESC stream consumer (background, retries until ROBOTDESC appears) ──
    // Start the gRPC server immediately so Monitor is ready before ROBOTDESC.
    // The ROBOTDESC connection retries in the background.
    {
        let svc_for_stream = svc.clone();
        let rules_for_stream = robotdesc_rules.clone();
        let mut reconnect_src = src.clone();
        let reconnect_consumer_id = cfg.id.clone();
        let reconnect_robotdesc_endpoint = cfg.robotdesc_endpoint.clone();
        tokio::spawn(async move {
            // Initial connect: retry until ROBOTDESC appears.
            let mut stream = loop {
                match robotdesc_ingest::open_robotdesc_stream(
                    &mut reconnect_src,
                    &reconnect_consumer_id,
                    reconnect_robotdesc_endpoint.as_deref(),
                )
                .await
                {
                    Ok(Some(s)) => break s,
                    Ok(None) => {
                        log::warn!("[monitor] waiting for Robotdesc health stream...");
                    }
                    Err(e) => {
                        log::warn!("[monitor] Robotdesc connect failed: {e:#} — retrying in 2s...");
                    }
                }
                tokio::time::sleep(std::time::Duration::from_secs(2)).await;
            };
            log::info!("[monitor] connected to Robotdesc StreamHealth");

            // Outer loop: reconnect on stream end or error.
            loop {
                // Inner loop: process stream messages.
                loop {
                    match stream.message().await {
                        Ok(Some(snapshot)) => {
                            let monitor = robotdesc_ingest::snapshot_to_monitor(
                                &snapshot,
                                &rules_for_stream,
                                monotonic_ns(),
                            );
                            svc_for_stream.update_snapshot(monitor).await;
                        }
                        Ok(None) => {
                            log::error!("[monitor] Robotdesc StreamHealth ended — reconnecting...");
                            break;
                        }
                        Err(e) => {
                            log::error!(
                                "[monitor] Robotdesc StreamHealth error: {e:#} — reconnecting..."
                            );
                            break;
                        }
                    }
                }

                // Reconnect with backoff.
                loop {
                    tokio::time::sleep(std::time::Duration::from_secs(2)).await;
                    match robotdesc_ingest::open_robotdesc_stream(
                        &mut reconnect_src,
                        &reconnect_consumer_id,
                        reconnect_robotdesc_endpoint.as_deref(),
                    )
                    .await
                    {
                        Ok(Some(new_stream)) => {
                            log::info!("[monitor] reconnected to Robotdesc StreamHealth");
                            stream = new_stream;
                            break;
                        }
                        Ok(None) => {
                            log::warn!("[monitor] Robotdesc still unavailable — retrying in 2s...");
                        }
                        Err(e) => {
                            log::warn!("[monitor] reconnect failed: {e:#} — retrying in 2s...");
                        }
                    }
                }
            }
        });
    }

    info!("Monitor gRPC on {listen_addr} (Robotdesc input)");
    eprintln!("robonix-monitor ready on {listen_addr} (Robotdesc input)");

    tonic::transport::Server::builder()
        .add_service(RobonixSystemMonitorGetServer::new(svc.clone()))
        .add_service(RobonixSystemMonitorStreamServer::new(svc.clone()))
        .add_service(RobonixSystemMonitorModulesGetServer::new(svc))
        .serve(listen_addr)
        .await
        .context("monitor gRPC server failed")?;

    Ok(())
}

fn monotonic_ns() -> u64 {
    static START: OnceLock<Instant> = OnceLock::new();
    START.get_or_init(Instant::now).elapsed().as_nanos() as u64
}
