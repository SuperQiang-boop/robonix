// SPDX-License-Identifier: MulanPSL-2.0
// Author: wheatfox <wheatfox17@icloud.com>
//
// Executor config — same three-source resolution as planner:
//   compiled defaults < YAML at $ROBONIX_CONFIG_PATH < CLI flags / env.

use anyhow::{Context, Result};
use clap::Parser;
use serde::Deserialize;
use std::path::{Path, PathBuf};

pub const DEFAULT_EXECUTOR_PROVIDER_ID: &str = "executor";
pub const EXECUTOR_NAMESPACE: &str = "robonix/system/executor";
pub const DEFAULT_SRC_ENDPOINT: &str = "127.0.0.1:50051";
pub const DEFAULT_LISTEN: &str = "127.0.0.1:50061";

#[derive(Debug, Clone)]
pub struct ExecutorConfig {
    pub src_endpoint: String,
    pub listen: String,
    pub id: String,
}

#[derive(Parser, Debug)]
#[command(
    name = "robonix-executor",
    about = "Robonix Executor — tool-call dispatch runtime"
)]
pub struct Args {
    /// Src control-plane endpoint. Also reads `ROBONIX_SRC` (the var rbnx /
    /// the Python API / gateway use) as an alias; see `env_src`.
    #[arg(long, env = "ROBONIX_SRC_ENDPOINT")]
    pub src: Option<String>,

    /// Address the SystemExecutor gRPC service binds to.
    #[arg(long, env = "ROBONIX_EXECUTOR_LISTEN")]
    pub listen: Option<String>,

    /// Override executor's id (singleton; rarely needed).
    #[arg(long, env = "ROBONIX_EXECUTOR_PROVIDER_ID")]
    pub id: Option<String>,

    /// Optional YAML config file (rbnx writes this; CLI/env still override).
    #[arg(long, env = "ROBONIX_CONFIG_PATH")]
    pub config: Option<PathBuf>,

    /// Log level for this component (`debug`/`info`/`warn`/`error`). Sets the
    /// log-file level floor; falls back to `ROBONIX_LOG_FILE_LEVEL` / `info`.
    /// Normally arrives inside `--config-json`, not as a standalone flag.
    #[arg(long)]
    pub log: Option<String>,

    /// The component's `system.executor` manifest block, serialized to JSON by
    /// rbnx and passed as one arg (`--config-json '{…}'`). Parsed by the binary
    /// itself — see `robonix_log::init_from_config`, which reads the `log`
    /// key from it so the manifest's per-component level reaches the log.
    #[arg(long)]
    pub config_json: Option<String>,
}

#[derive(Default, Deserialize)]
struct FileConfig {
    #[serde(default)]
    src_endpoint: Option<String>,
    #[serde(default)]
    listen: Option<String>,
    #[serde(default)]
    id: Option<String>,
}

impl ExecutorConfig {
    pub fn resolve(args: Args) -> Result<Self> {
        let file_cfg: FileConfig = match &args.config {
            Some(path) => load_yaml(path)?,
            None => FileConfig::default(),
        };
        Ok(Self {
            src_endpoint: args
                .src
                .or_else(env_src)
                .or(file_cfg.src_endpoint)
                .unwrap_or_else(|| DEFAULT_SRC_ENDPOINT.to_string()),
            listen: args
                .listen
                .or(file_cfg.listen)
                .unwrap_or_else(|| DEFAULT_LISTEN.to_string()),
            id: args
                .id
                .or(file_cfg.id)
                .unwrap_or_else(|| DEFAULT_EXECUTOR_PROVIDER_ID.to_string()),
        })
    }
}

/// Read the `ROBONIX_SRC` env var as an src-endpoint alias.
///
/// rbnx, the Python API, and gateway all configure the src endpoint via
/// `ROBONIX_SRC`, while executor/planner historically only honored
/// `ROBONIX_SRC_ENDPOINT` (the clap `env`). Accepting `ROBONIX_SRC` here as
/// well means a single env var configures every component. Without it, setting
/// only `ROBONIX_SRC` left executor silently falling back to
/// `DEFAULT_SRC_ENDPOINT` (127.0.0.1:50051) — it would then dial the wrong
/// src and log 127.0.0.1 even after the operator "changed" the endpoint.
/// Empty values are ignored so an exported-but-blank var doesn't shadow later
/// sources.
fn env_src() -> Option<String> {
    std::env::var("ROBONIX_SRC").ok().filter(|v| !v.is_empty())
}

fn load_yaml(path: &Path) -> Result<FileConfig> {
    let raw = std::fs::read_to_string(path)
        .with_context(|| format!("read executor config '{}'", path.display()))?;
    serde_yaml::from_str(&raw)
        .with_context(|| format!("parse executor config '{}'", path.display()))
}
