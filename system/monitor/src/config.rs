// SPDX-License-Identifier: MulanPSL-2.0
//
// Monitor config resolution:
//   compiled defaults < YAML < rbnx manifest JSON < CLI flags / env.

use anyhow::{Context, Result};
use clap::Parser;
use serde::Deserialize;
use std::path::{Path, PathBuf};

/// Default Src provider id registered by Monitor.
pub const DEFAULT_MONITOR_PROVIDER_ID: &str = "monitor";
/// Src service namespace for Monitor capability declarations.
pub const MONITOR_NAMESPACE: &str = "robonix/system/monitor";
/// Default Src control-plane endpoint.
pub const DEFAULT_SRC_ENDPOINT: &str = "127.0.0.1:50051";
/// Default Monitor gRPC listen address.
pub const DEFAULT_LISTEN: &str = "127.0.0.1:50091";
/// Default mock Robotdesc gRPC listen address.
pub const DEFAULT_MOCK_ROBOTDESC_LISTEN: &str = "127.0.0.1:50092";
/// Default mock Robotdesc stream update interval in milliseconds.
pub const DEFAULT_MOCK_ROBOTDESC_INTERVAL_MS: u64 = 10_000;
/// Default Python binary for hardware bridge subprocesses.
pub const DEFAULT_BRIDGE_PYTHON: &str = "python3";
/// Default module health report TTL in milliseconds.
pub const DEFAULT_MODULE_HEALTH_TTL_MS: u32 = 5_000;
/// Executor module health capability.
pub const EXECUTOR_GET_HEALTH_CONTRACT: &str = "robonix/system/executor/get_health";
/// Planner module health capability.
pub const PLANNER_GET_HEALTH_CONTRACT: &str = "robonix/system/planner/get_health";

/// Mock Robotdesc arm data source: fully synthetic, or real hardware via bridge.
#[derive(Debug, Clone)]
pub enum MockArmConfig {
    /// Fully synthetic arm data — no bridge subprocess.
    Synthetic,
    /// Real Piper arm via piper_sdk on a CAN bus.
    Piper {
        can_port: String,
        python_bin: String,
        script: PathBuf,
    },
    /// Real Koch arm via dynamixel_sdk on a serial port.
    Koch {
        serial_port: String,
        python_bin: String,
        script: PathBuf,
    },
}

impl MockArmConfig {
    /// Short label for logging / display.
    pub fn label(&self) -> &'static str {
        match self {
            Self::Synthetic => "synthetic",
            Self::Piper { .. } => "piper",
            Self::Koch { .. } => "koch",
        }
    }
}

/// Deployment policy for a module that Monitor expects to supervise.
#[derive(Debug, Clone, Copy, Default, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "lowercase")]
pub enum ExpectedModulePolicy {
    Required,
    #[default]
    Optional,
    Disabled,
}

/// One module that Monitor should include in the module-health view.
#[derive(Debug, Clone, Deserialize, PartialEq, Eq)]
pub struct ExpectedModuleConfig {
    pub module_id: String,
    #[serde(default)]
    pub provider_id: Option<String>,
    #[serde(default)]
    pub capability: Option<String>,
    #[serde(default)]
    pub policy: ExpectedModulePolicy,
    #[serde(default = "default_module_health_ttl_ms")]
    pub ttl_ms: u32,
}

impl ExpectedModuleConfig {
    /// Return the configured provider identity, falling back to the module identity.
    pub fn module_key(&self) -> String {
        self.provider_id
            .as_deref()
            .map(str::trim)
            .filter(|provider_id| !provider_id.is_empty())
            .unwrap_or_else(|| self.module_id.trim())
            .to_string()
    }

    pub fn provider_id_or_empty(&self) -> &str {
        self.provider_id
            .as_deref()
            .map(str::trim)
            .unwrap_or_default()
    }
}

/// Resolved Monitor configuration: defaults < YAML < manifest JSON < CLI/env.
#[derive(Debug, Clone)]
pub struct MonitorConfig {
    pub src_endpoint: String,
    pub listen: String,
    pub id: String,
    pub thresholds_path: PathBuf,
    pub robotdesc_endpoint: Option<String>,
    pub mock_robotdesc: bool,
    pub mock_robotdesc_id: String,
    pub mock_robotdesc_listen: String,
    pub mock_robotdesc_scenario: String,
    pub mock_robotdesc_interval_ms: u64,
    /// Mock arm data source (default: Synthetic).
    pub mock_robotdesc_arm: MockArmConfig,
    /// Modules Monitor should supervise in the module-health path.
    pub expected_modules: Vec<ExpectedModuleConfig>,
}

#[derive(Parser, Debug)]
#[command(
    name = "robonix-monitor",
    about = "Robonix Monitor — health monitoring: power state, component health, threshold alerts"
)]
pub struct Args {
    /// Src control-plane endpoint.
    #[arg(long, env = "ROBONIX_SRC_ENDPOINT")]
    pub src: Option<String>,

    /// Address the Monitor gRPC services bind to.
    #[arg(long, env = "ROBONIX_MONITOR_LISTEN")]
    pub listen: Option<String>,

    /// Override monitor's provider id (singleton; rarely needed).
    #[arg(long, env = "ROBONIX_MONITOR_PROVIDER_ID")]
    pub id: Option<String>,

    /// Path to the board threshold YAML (e.g. thresholds/example_thresholds.yaml).
    #[arg(long, env = "ROBONIX_MONITOR_THRESHOLDS_PATH")]
    pub thresholds_path: Option<PathBuf>,

    /// Optional Robotdesc gRPC endpoint. When set, Monitor consumes RobotdescHealthSnapshot.
    #[arg(long, env = "ROBONIX_ROBOTDESC_ENDPOINT")]
    pub robotdesc_endpoint: Option<String>,

    /// Run this binary as a mock Robotdesc server instead of Monitor.
    #[arg(long, env = "ROBONIX_MONITOR_MOCK_ROBOTDESC", default_value_t = false)]
    pub mock_robotdesc: bool,

    /// Provider id used when --mock-robotdesc registers with Src.
    #[arg(long, env = "ROBONIX_MONITOR_MOCK_ROBOTDESC_ID")]
    pub mock_robotdesc_id: Option<String>,

    /// Address the mock Robotdesc gRPC services bind to.
    #[arg(long, env = "ROBONIX_MONITOR_MOCK_ROBOTDESC_LISTEN")]
    pub mock_robotdesc_listen: Option<String>,

    /// Mock Robotdesc scenario: normal, ramp, fault, toggle, or mixed.
    #[arg(long, env = "ROBONIX_MONITOR_MOCK_ROBOTDESC_SCENARIO")]
    pub mock_robotdesc_scenario: Option<String>,

    /// Mock Robotdesc stream update interval in milliseconds.
    #[arg(long, env = "ROBONIX_MONITOR_MOCK_ROBOTDESC_INTERVAL_MS")]
    pub mock_robotdesc_interval_ms: Option<u64>,

    /// Mock arm data source: synthetic (default), piper, or koch.
    #[arg(long, env = "ROBONIX_MONITOR_MOCK_ROBOTDESC_ARM")]
    pub mock_robotdesc_arm: Option<String>,

    /// CAN port for real Piper hardware (e.g. "can0"). Only used with --mock-robotdesc-arm=piper.
    #[arg(long, env = "ROBONIX_MONITOR_MOCK_ROBOTDESC_PIPER_CAN")]
    pub mock_robotdesc_piper_can: Option<String>,

    /// Serial port for real Koch hardware (e.g. "/dev/ttyUSB0"). Only used with --mock-robotdesc-arm=koch.
    #[arg(long, env = "ROBONIX_MONITOR_MOCK_ROBOTDESC_KOCH_PORT")]
    pub mock_robotdesc_koch_port: Option<String>,

    /// Python binary for hardware bridge subprocesses.
    #[arg(long, env = "ROBONIX_MONITOR_MOCK_ROBOTDESC_BRIDGE_PYTHON")]
    pub mock_robotdesc_bridge_python: Option<String>,

    /// Path to piper_bridge.py script.
    #[arg(long, env = "ROBONIX_MONITOR_MOCK_ROBOTDESC_PIPER_SCRIPT")]
    pub mock_robotdesc_piper_script: Option<PathBuf>,

    /// Path to koch_bridge.py script.
    #[arg(long, env = "ROBONIX_MONITOR_MOCK_ROBOTDESC_KOCH_SCRIPT")]
    pub mock_robotdesc_koch_script: Option<PathBuf>,

    /// Optional YAML config file (rbnx writes this; CLI/env still override).
    #[arg(long, env = "ROBONIX_CONFIG_PATH")]
    pub config: Option<PathBuf>,

    /// The `system.monitor` manifest block serialized to JSON by rbnx.
    #[arg(long)]
    pub config_json: Option<String>,

    /// Log filter (env_logger syntax; e.g. `info`, `robonix_monitor=debug`).
    /// Default: `robonix_monitor=info`. Falls back to `RUST_LOG` if unset.
    #[arg(long)]
    pub log: Option<String>,
}

#[derive(Default, Deserialize)]
struct FileConfig {
    #[serde(default, alias = "src")]
    src_endpoint: Option<String>,
    #[serde(default)]
    listen: Option<String>,
    #[serde(default)]
    id: Option<String>,
    #[serde(default)]
    thresholds_path: Option<PathBuf>,
    #[serde(default)]
    robotdesc_endpoint: Option<String>,
    #[serde(default)]
    mock_robotdesc_id: Option<String>,
    #[serde(default)]
    mock_robotdesc_listen: Option<String>,
    #[serde(default)]
    mock_robotdesc_scenario: Option<String>,
    #[serde(default)]
    mock_robotdesc_interval_ms: Option<u64>,
    #[serde(default)]
    mock_robotdesc_arm: Option<String>,
    #[serde(default)]
    mock_robotdesc_piper_can: Option<String>,
    #[serde(default)]
    mock_robotdesc_koch_port: Option<String>,
    #[serde(default)]
    mock_robotdesc_bridge_python: Option<String>,
    #[serde(default)]
    mock_robotdesc_piper_script: Option<PathBuf>,
    #[serde(default)]
    mock_robotdesc_koch_script: Option<PathBuf>,
    #[serde(default)]
    expected_modules: Option<Vec<ExpectedModuleConfig>>,
}

impl MonitorConfig {
    /// Resolve defaults, optional YAML, manifest JSON, and CLI/env values.
    /// Priority: CLI/env > manifest JSON > YAML file > compiled defaults.
    pub fn resolve(args: Args) -> Result<Self> {
        let file_cfg: FileConfig = match &args.config {
            Some(path) => load_yaml(path)?,
            None => FileConfig::default(),
        };
        let manifest_cfg: FileConfig = match args.config_json.as_deref() {
            Some(raw) => serde_json::from_str(raw).context("parse monitor --config-json")?,
            None => FileConfig::default(),
        };

        let thresholds_dir = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("thresholds");
        let default_thresholds = thresholds_dir.join("example_thresholds.yaml");

        let arm_kind = args
            .mock_robotdesc_arm
            .or(manifest_cfg.mock_robotdesc_arm)
            .or(file_cfg.mock_robotdesc_arm)
            .unwrap_or_else(|| "synthetic".to_string());

        let bridge_python = args
            .mock_robotdesc_bridge_python
            .or(manifest_cfg.mock_robotdesc_bridge_python)
            .or(file_cfg.mock_robotdesc_bridge_python)
            .unwrap_or_else(|| DEFAULT_BRIDGE_PYTHON.to_string());

        let mock_robotdesc_arm = match arm_kind.trim().to_ascii_lowercase().as_str() {
            "piper" => {
                let can = args
                    .mock_robotdesc_piper_can
                    .or(manifest_cfg.mock_robotdesc_piper_can)
                    .or(file_cfg.mock_robotdesc_piper_can)
                    .unwrap_or_else(|| "can0".to_string());
                let script = args
                    .mock_robotdesc_piper_script
                    .or(manifest_cfg.mock_robotdesc_piper_script)
                    .or(file_cfg.mock_robotdesc_piper_script)
                    .unwrap_or_else(|| {
                        PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("scripts/piper_bridge.py")
                    });
                MockArmConfig::Piper {
                    can_port: can,
                    python_bin: bridge_python,
                    script,
                }
            }
            "koch" => {
                let port = args
                    .mock_robotdesc_koch_port
                    .or(manifest_cfg.mock_robotdesc_koch_port)
                    .or(file_cfg.mock_robotdesc_koch_port)
                    .unwrap_or_else(|| "/dev/ttyUSB0".to_string());
                let script = args
                    .mock_robotdesc_koch_script
                    .or(manifest_cfg.mock_robotdesc_koch_script)
                    .or(file_cfg.mock_robotdesc_koch_script)
                    .unwrap_or_else(|| {
                        PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("scripts/koch_bridge.py")
                    });
                MockArmConfig::Koch {
                    serial_port: port,
                    python_bin: bridge_python,
                    script,
                }
            }
            other => {
                if other != "synthetic" {
                    log::warn!(
                        "[monitor] unrecognized --mock-robotdesc-arm '{}', using synthetic",
                        other
                    );
                }
                MockArmConfig::Synthetic
            }
        };

        let id = args
            .id
            .or(manifest_cfg.id)
            .or(file_cfg.id)
            .unwrap_or_else(|| DEFAULT_MONITOR_PROVIDER_ID.to_string());
        let expected_modules = manifest_cfg
            .expected_modules
            .or(file_cfg.expected_modules)
            .unwrap_or_else(|| default_expected_modules(&id));
        validate_expected_modules(&expected_modules)?;

        Ok(Self {
            src_endpoint: args
                .src
                .or(manifest_cfg.src_endpoint)
                .or(file_cfg.src_endpoint)
                .unwrap_or_else(|| DEFAULT_SRC_ENDPOINT.to_string()),
            listen: args
                .listen
                .or(manifest_cfg.listen)
                .or(file_cfg.listen)
                .unwrap_or_else(|| DEFAULT_LISTEN.to_string()),
            id,
            thresholds_path: args
                .thresholds_path
                .or(manifest_cfg.thresholds_path)
                .or(file_cfg.thresholds_path)
                .unwrap_or(default_thresholds),
            robotdesc_endpoint: args
                .robotdesc_endpoint
                .or(manifest_cfg.robotdesc_endpoint)
                .or(file_cfg.robotdesc_endpoint),
            mock_robotdesc: args.mock_robotdesc,
            mock_robotdesc_id: args
                .mock_robotdesc_id
                .or(manifest_cfg.mock_robotdesc_id)
                .or(file_cfg.mock_robotdesc_id)
                .unwrap_or_else(|| "mock-robotdesc".to_string()),
            mock_robotdesc_listen: args
                .mock_robotdesc_listen
                .or(manifest_cfg.mock_robotdesc_listen)
                .or(file_cfg.mock_robotdesc_listen)
                .unwrap_or_else(|| DEFAULT_MOCK_ROBOTDESC_LISTEN.to_string()),
            mock_robotdesc_scenario: args
                .mock_robotdesc_scenario
                .or(manifest_cfg.mock_robotdesc_scenario)
                .or(file_cfg.mock_robotdesc_scenario)
                .unwrap_or_else(|| "normal".to_string()),
            mock_robotdesc_interval_ms: args
                .mock_robotdesc_interval_ms
                .or(manifest_cfg.mock_robotdesc_interval_ms)
                .or(file_cfg.mock_robotdesc_interval_ms)
                .unwrap_or(DEFAULT_MOCK_ROBOTDESC_INTERVAL_MS),
            mock_robotdesc_arm,
            expected_modules,
        })
    }
}

/// Supervise Monitor itself plus the two system modules with typed health clients.
pub fn default_expected_modules(monitor_provider_id: &str) -> Vec<ExpectedModuleConfig> {
    vec![
        ExpectedModuleConfig {
            module_id: "monitor".to_string(),
            provider_id: Some(monitor_provider_id.to_string()),
            capability: None,
            policy: ExpectedModulePolicy::Required,
            ttl_ms: 0,
        },
        ExpectedModuleConfig {
            module_id: "executor".to_string(),
            provider_id: Some("executor".to_string()),
            capability: Some(EXECUTOR_GET_HEALTH_CONTRACT.to_string()),
            policy: ExpectedModulePolicy::Required,
            ttl_ms: DEFAULT_MODULE_HEALTH_TTL_MS,
        },
        ExpectedModuleConfig {
            module_id: "planner".to_string(),
            provider_id: Some("planner".to_string()),
            capability: Some(PLANNER_GET_HEALTH_CONTRACT.to_string()),
            policy: ExpectedModulePolicy::Required,
            ttl_ms: DEFAULT_MODULE_HEALTH_TTL_MS,
        },
    ]
}

fn default_module_health_ttl_ms() -> u32 {
    DEFAULT_MODULE_HEALTH_TTL_MS
}

fn validate_expected_modules(modules: &[ExpectedModuleConfig]) -> Result<()> {
    for module in modules {
        if module.module_id.trim().is_empty() {
            anyhow::bail!("expected_modules contains an empty module_id");
        }
    }
    Ok(())
}

fn load_yaml(path: &Path) -> Result<FileConfig> {
    let raw = std::fs::read_to_string(path)
        .with_context(|| format!("read monitor config '{}'", path.display()))?;
    serde_yaml::from_str(&raw).with_context(|| format!("parse monitor config '{}'", path.display()))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn args_defaults() {
        let args = Args::try_parse_from(["robonix-monitor"]).expect("parse empty args");
        assert!(!args.mock_robotdesc);
        assert!(args.src.is_none());
        assert!(args.listen.is_none());
        assert!(args.robotdesc_endpoint.is_none());
        assert!(args.config_json.is_none());
    }

    #[test]
    fn args_mock_robotdesc_flags() {
        let args = Args::try_parse_from([
            "robonix-monitor",
            "--mock-robotdesc",
            "--mock-robotdesc-scenario",
            "ramp",
            "--mock-robotdesc-interval-ms",
            "5000",
        ])
        .expect("parse mock robotdesc args");
        assert!(args.mock_robotdesc);
        assert_eq!(args.mock_robotdesc_scenario.unwrap(), "ramp");
        assert_eq!(args.mock_robotdesc_interval_ms.unwrap(), 5000);
    }

    #[test]
    fn args_piper_arm_flags() {
        let args = Args::try_parse_from([
            "robonix-monitor",
            "--mock-robotdesc",
            "--mock-robotdesc-arm",
            "piper",
            "--mock-robotdesc-piper-can",
            "can0",
            "--mock-robotdesc-bridge-python",
            "/usr/bin/python3",
        ])
        .expect("parse piper arm args");
        assert_eq!(args.mock_robotdesc_arm.unwrap(), "piper");
        assert_eq!(args.mock_robotdesc_piper_can.unwrap(), "can0");
        assert_eq!(
            args.mock_robotdesc_bridge_python.unwrap(),
            "/usr/bin/python3"
        );
    }

    #[test]
    fn args_koch_arm_flags() {
        let args = Args::try_parse_from([
            "robonix-monitor",
            "--mock-robotdesc",
            "--mock-robotdesc-arm",
            "koch",
            "--mock-robotdesc-koch-port",
            "/dev/ttyUSB0",
        ])
        .expect("parse koch arm args");
        assert_eq!(args.mock_robotdesc_arm.unwrap(), "koch");
        assert_eq!(args.mock_robotdesc_koch_port.unwrap(), "/dev/ttyUSB0");
    }

    #[test]
    fn resolve_defaults_without_yaml() {
        let args = Args::try_parse_from(["robonix-monitor"]).unwrap();
        let cfg = MonitorConfig::resolve(args).expect("resolve defaults");
        assert_eq!(cfg.id, DEFAULT_MONITOR_PROVIDER_ID);
        assert_eq!(cfg.src_endpoint, DEFAULT_SRC_ENDPOINT);
        assert_eq!(cfg.listen, DEFAULT_LISTEN);
        assert_eq!(cfg.mock_robotdesc_scenario, "normal");
        assert_eq!(
            cfg.mock_robotdesc_interval_ms,
            DEFAULT_MOCK_ROBOTDESC_INTERVAL_MS
        );
        assert!(cfg.robotdesc_endpoint.is_none());
        assert!(!cfg.mock_robotdesc);
        assert!(matches!(cfg.mock_robotdesc_arm, MockArmConfig::Synthetic));
        assert_eq!(cfg.expected_modules.len(), 3);
        assert_eq!(cfg.expected_modules[0].module_id, "monitor");
        assert_eq!(
            cfg.expected_modules[0].policy,
            ExpectedModulePolicy::Required
        );
        assert_eq!(cfg.expected_modules[1].module_id, "executor");
        assert_eq!(
            cfg.expected_modules[1].policy,
            ExpectedModulePolicy::Required
        );
        assert_eq!(cfg.expected_modules[2].module_id, "planner");
        assert_eq!(
            cfg.expected_modules[2].policy,
            ExpectedModulePolicy::Required
        );
    }

    #[test]
    fn resolve_cli_overrides_yaml_defaults() {
        let args = Args::try_parse_from([
            "robonix-monitor",
            "--id",
            "custom-monitor",
            "--listen",
            "0.0.0.0:9999",
            "--robotdesc-endpoint",
            "10.0.0.1:50092",
        ])
        .unwrap();
        let cfg = MonitorConfig::resolve(args).expect("resolve with overrides");
        assert_eq!(cfg.id, "custom-monitor");
        assert_eq!(cfg.listen, "0.0.0.0:9999");
        assert_eq!(cfg.robotdesc_endpoint.unwrap(), "10.0.0.1:50092");
    }

    /// Verifies that rbnx's manifest JSON configures scalar fields and the
    /// nested module supervision policy that has no standalone CLI flag.
    #[test]
    fn resolve_manifest_json() {
        let manifest_json = r#"{
            "src": "10.0.0.2:50051",
            "listen": "127.0.0.1:50093",
            "id": "webots-monitor",
            "thresholds_path": "/tmp/monitor-thresholds.yaml",
            "expected_modules": [
                {
                    "module_id": "executor",
                    "provider_id": "executor-main",
                    "policy": "required",
                    "ttl_ms": 2500
                }
            ]
        }"#;
        let args = Args::try_parse_from(["robonix-monitor", "--config-json", manifest_json])
            .expect("parse manifest JSON arg");

        let cfg = MonitorConfig::resolve(args).expect("resolve manifest JSON");
        assert_eq!(cfg.src_endpoint, "10.0.0.2:50051");
        assert_eq!(cfg.listen, "127.0.0.1:50093");
        assert_eq!(cfg.id, "webots-monitor");
        assert_eq!(
            cfg.thresholds_path,
            PathBuf::from("/tmp/monitor-thresholds.yaml")
        );
        assert_eq!(cfg.expected_modules.len(), 1);
        assert_eq!(cfg.expected_modules[0].module_key(), "executor-main");
        assert_eq!(cfg.expected_modules[0].ttl_ms, 2500);
    }

    /// Verifies that explicit CLI values remain authoritative over values
    /// delivered in the manifest JSON by rbnx.
    #[test]
    fn resolve_cli_overrides_manifest_json() {
        let args = Args::try_parse_from([
            "robonix-monitor",
            "--config-json",
            r#"{"id":"manifest-monitor","listen":"127.0.0.1:50093"}"#,
            "--id",
            "cli-monitor",
            "--listen",
            "127.0.0.1:50094",
        ])
        .expect("parse CLI and manifest args");

        let cfg = MonitorConfig::resolve(args).expect("resolve CLI overrides");
        assert_eq!(cfg.id, "cli-monitor");
        assert_eq!(cfg.listen, "127.0.0.1:50094");
        assert_eq!(
            cfg.expected_modules[0].provider_id_or_empty(),
            "cli-monitor"
        );
    }

    #[test]
    fn resolve_piper_arm_config() {
        let args = Args::try_parse_from([
            "robonix-monitor",
            "--mock-robotdesc-arm",
            "piper",
            "--mock-robotdesc-piper-can",
            "can1",
        ])
        .unwrap();
        let cfg = MonitorConfig::resolve(args).expect("resolve piper arm");
        match &cfg.mock_robotdesc_arm {
            MockArmConfig::Piper { can_port, .. } => assert_eq!(can_port, "can1"),
            other => panic!("expected Piper, got {:?}", other.label()),
        }
    }

    #[test]
    fn resolve_koch_arm_config() {
        let args = Args::try_parse_from([
            "robonix-monitor",
            "--mock-robotdesc-arm",
            "koch",
            "--mock-robotdesc-koch-port",
            "/dev/ttyUSB1",
        ])
        .unwrap();
        let cfg = MonitorConfig::resolve(args).expect("resolve koch arm");
        match &cfg.mock_robotdesc_arm {
            MockArmConfig::Koch { serial_port, .. } => assert_eq!(serial_port, "/dev/ttyUSB1"),
            other => panic!("expected Koch, got {:?}", other.label()),
        }
    }

    #[test]
    fn file_config_parses_expected_modules() {
        let file_cfg: FileConfig = serde_yaml::from_str(
            r#"
expected_modules:
  - module_id: executor
    provider_id: executor-main
    capability: robonix/system/executor/get_health
    policy: required
    ttl_ms: 3000
  - module_id: speech
    policy: disabled
"#,
        )
        .expect("parse expected modules");

        let modules = file_cfg.expected_modules.expect("expected modules");
        assert_eq!(modules.len(), 2);
        assert_eq!(modules[0].module_key(), "executor-main");
        assert_eq!(modules[0].policy, ExpectedModulePolicy::Required);
        assert_eq!(modules[0].ttl_ms, 3000);
        assert_eq!(modules[1].module_key(), "speech");
        assert_eq!(modules[1].policy, ExpectedModulePolicy::Disabled);
        assert_eq!(modules[1].ttl_ms, DEFAULT_MODULE_HEALTH_TTL_MS);
    }
}
