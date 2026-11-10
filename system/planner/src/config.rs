// SPDX-License-Identifier: MulanPSL-2.0
// Author: wheatfox <wheatfox17@icloud.com>
//
// Planner config: how a launched planner process figures out where src is,
// what address to bind, and how to reach the LLM upstream.
//
// Three sources, from lowest to highest priority:
//   1. compiled defaults (src endpoint, listen address, id, …)
//   2. optional YAML at `$ROBONIX_CONFIG_PATH` or `--config <path>`
//      (used by `rbnx boot` to write a slice of `system.planner` from
//      `robonix_manifest.yaml`)
//   3. CLI flags / per-field env vars
//
// Higher-priority source overrides lower. Manual launch only needs the
// minimum: an src endpoint, a VLM upstream URL, and an API key.

use anyhow::{Context, Result, bail};
use clap::Parser;
use serde::Deserialize;
use std::path::{Path, PathBuf};

pub const DEFAULT_PLANNER_PROVIDER_ID: &str = "planner";
pub const PLANNER_NAMESPACE: &str = "robonix/system/planner";
pub const DEFAULT_SRC_ENDPOINT: &str = "127.0.0.1:50051";
pub const DEFAULT_LISTEN: &str = "127.0.0.1:50071";
pub const DEFAULT_VLM_FORMAT: &str = "openai";

/// Fully-resolved settings the planner binary runs against.
#[derive(Debug, Clone)]
pub struct PlannerConfig {
    pub src_endpoint: String,
    pub listen: String,
    pub id: String,
    pub vlm: VlmConfig,
}

#[derive(Debug, Clone)]
pub struct VlmConfig {
    pub upstream: String,
    pub api_key: String,
    pub model: String,
    /// Wire dialect. Currently only "openai" is implemented; checked at
    /// `resolve` time, kept on the struct for diagnostics / future routing.
    #[allow(dead_code)]
    pub api_format: String,
}

/// CLI surface; every field is optional so config-file mode stays usable
/// without spelling out flags. clap also reads the listed env vars.
#[derive(Parser, Debug)]
#[command(name = "robonix-planner", about = "Robonix Planner — VLM planner")]
pub struct Args {
    /// Src control-plane endpoint. Also reads `ROBONIX_SRC` (the var rbnx /
    /// the Python API / gateway use) as an alias; see `env_src`.
    #[arg(long, env = "ROBONIX_SRC_ENDPOINT")]
    pub src: Option<String>,

    /// Address planner's SystemPlanner gRPC binds to.
    #[arg(long, env = "ROBONIX_PLANNER_LISTEN")]
    pub listen: Option<String>,

    /// Override planner's id (singleton, rarely needed).
    #[arg(long, env = "ROBONIX_PLANNER_PROVIDER_ID")]
    pub id: Option<String>,

    /// LLM API base URL (e.g. "https://api.openai.com/v1").
    #[arg(long, env = "ROBONIX_VLM_UPSTREAM")]
    pub vlm_upstream: Option<String>,

    /// LLM API key.
    #[arg(long, env = "ROBONIX_VLM_API_KEY")]
    pub vlm_api_key: Option<String>,

    /// LLM model identifier.
    #[arg(long, env = "ROBONIX_VLM_MODEL")]
    pub vlm_model: Option<String>,

    /// LLM API dialect ("openai" only for now).
    #[arg(long, env = "ROBONIX_VLM_FORMAT")]
    pub vlm_format: Option<String>,

    /// YAML config file (rbnx writes this; CLI/env still override individual fields).
    #[arg(long, env = "ROBONIX_CONFIG_PATH")]
    pub config: Option<PathBuf>,

    /// Log level for this component (`debug`/`info`/`warn`/`error`). Sets the
    /// log-file level floor; falls back to `ROBONIX_LOG_FILE_LEVEL` / `info`.
    /// Normally arrives inside `--config-json`, not as a standalone flag.
    #[arg(long)]
    pub log: Option<String>,

    /// The component's `system.planner` manifest block, serialized to JSON by
    /// rbnx and passed as one arg (`--config-json '{…}'`). Parsed by the
    /// binary itself — see `robonix_log::init_from_config`, which reads the
    /// `log` key from it so the manifest's per-component level reaches the log.
    #[arg(long)]
    pub config_json: Option<String>,
}

/// Optional YAML schema. Field names match `PlannerConfig` (flat) so a
/// hand-written file looks like the manifest's `system.planner` block.
#[derive(Default, Deserialize)]
struct FileConfig {
    #[serde(default)]
    src_endpoint: Option<String>,
    #[serde(default)]
    listen: Option<String>,
    #[serde(default)]
    id: Option<String>,
    #[serde(default)]
    vlm: Option<FileVlmConfig>,
}

#[derive(Default, Deserialize)]
struct FileVlmConfig {
    #[serde(default)]
    upstream: Option<String>,
    #[serde(default)]
    api_key: Option<String>,
    #[serde(default)]
    model: Option<String>,
    #[serde(default)]
    api_format: Option<String>,
}

impl PlannerConfig {
    /// Build the resolved config from CLI args (which already pulled env
    /// vars). Reads optional YAML; CLI/env still override file fields.
    pub fn resolve(args: Args) -> Result<Self> {
        let file_cfg: FileConfig = match &args.config {
            Some(path) => load_yaml(path)?,
            None => FileConfig::default(),
        };
        let file_vlm = file_cfg.vlm.unwrap_or_default();

        let src_endpoint = args
            .src
            .or_else(env_src)
            .or(file_cfg.src_endpoint)
            .unwrap_or_else(|| DEFAULT_SRC_ENDPOINT.to_string());
        let listen = args
            .listen
            .or(file_cfg.listen)
            .unwrap_or_else(|| DEFAULT_LISTEN.to_string());
        let id = args
            .id
            .or(file_cfg.id)
            .unwrap_or_else(|| DEFAULT_PLANNER_PROVIDER_ID.to_string());
        let api_format = args
            .vlm_format
            .or(file_vlm.api_format)
            .unwrap_or_else(|| DEFAULT_VLM_FORMAT.to_string());
        if api_format != "openai" {
            bail!("vlm api_format='{api_format}' not supported (only 'openai')");
        }

        let upstream = args
            .vlm_upstream
            .or(file_vlm.upstream)
            .filter(|s| !s.trim().is_empty())
            .ok_or_else(|| {
                missing_field("vlm.upstream", "ROBONIX_VLM_UPSTREAM", "--vlm-upstream")
            })?;
        let api_key = args
            .vlm_api_key
            .or(file_vlm.api_key)
            .filter(|s| !s.trim().is_empty())
            .ok_or_else(|| missing_field("vlm.api_key", "ROBONIX_VLM_API_KEY", "--vlm-api-key"))?;
        let model = args
            .vlm_model
            .or(file_vlm.model)
            .filter(|s| !s.trim().is_empty())
            .ok_or_else(|| missing_field("vlm.model", "ROBONIX_VLM_MODEL", "--vlm-model"))?;

        Ok(Self {
            src_endpoint,
            listen,
            id,
            vlm: VlmConfig {
                upstream,
                api_key,
                model,
                api_format,
            },
        })
    }
}

/// Read the `ROBONIX_SRC` env var as an src-endpoint alias.
///
/// rbnx, the Python API, and gateway all configure the src endpoint via
/// `ROBONIX_SRC`, while executor/planner historically only honored
/// `ROBONIX_SRC_ENDPOINT` (the clap `env`). Accepting `ROBONIX_SRC` here as
/// well means a single env var configures every component. Without it, setting
/// only `ROBONIX_SRC` left planner silently falling back to
/// `DEFAULT_SRC_ENDPOINT` (127.0.0.1:50051) — it would then dial the wrong
/// src and log 127.0.0.1 even after the operator "changed" the endpoint.
/// Empty values are ignored so an exported-but-blank var doesn't shadow later
/// sources.
fn env_src() -> Option<String> {
    std::env::var("ROBONIX_SRC").ok().filter(|v| !v.is_empty())
}

fn load_yaml(path: &Path) -> Result<FileConfig> {
    let raw = std::fs::read_to_string(path)
        .with_context(|| format!("read planner config '{}'", path.display()))?;
    serde_yaml::from_str(&raw).with_context(|| format!("parse planner config '{}'", path.display()))
}

fn missing_field(yaml_path: &str, env_var: &str, flag: &str) -> anyhow::Error {
    anyhow::anyhow!(
        "missing required field '{yaml_path}': set it in --config YAML, env {env_var}, or pass {flag}"
    )
}
