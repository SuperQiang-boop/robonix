// SPDX-License-Identifier: MulanPSL-2.0
//
// Robotdesc runtime config. Flat and small: one process, one robot, one
// deployment. The v1 shape (with `deployments: Vec`, `robonix_root`,
// `default_robot`, `rbnx_bin`, `start_packages`) was over-general —
// there is exactly one robotdesc per `rbnx boot`, exactly one manifest it
// serves, and exactly one robot inside that manifest's robotdesc.yaml. The
// current shape reflects that.
//
// Resolution order (highest wins):
//   1. CLI flags / env vars
//   2. `--config <yaml>`   (operator-supplied file; not shipped bundled)
//   3. built-in defaults

use anyhow::{Context, Result, bail};
use clap::Parser;
use serde::Deserialize;
use std::path::{Component, Path, PathBuf};

pub const DEFAULT_PROVIDER_ID: &str = "robotdesc";
pub const DEFAULT_SRC_ENDPOINT: &str = "127.0.0.1:50051";
pub const DEFAULT_LISTEN: &str = "127.0.0.1:50091";

#[derive(Debug, Clone)]
pub struct RobotdescConfig {
    pub src_endpoint: String,
    pub listen: String,
    pub provider_id: String,
    /// Absolute path to this deployment's `robotdesc.yaml` (or a compatibly
    /// named file such as `robonix.robotdesc.yaml`). Robotdesc reads three things
    /// from its parent directory:
    ///   * this file, for robot id and URDF reference
    ///   * the URDF pointed at by `urdf.path` (resolved relative to
    ///     this file)
    pub robot_yaml: PathBuf,
    /// Exact deploy manifest selected by `rbnx boot -f`. Defaults to the
    /// standard manifest next to `robot_yaml`.
    pub deployment_manifest: PathBuf,
}

#[derive(Parser, Debug)]
#[command(
    name = "robonix-robotdesc",
    about = "Robonix Robotdesc raw body service"
)]
pub struct Args {
    #[arg(long, env = "ROBONIX_SRC_ENDPOINT")]
    pub src: Option<String>,

    #[arg(long, env = "ROBONIX_ROBOTDESC_LISTEN")]
    pub listen: Option<String>,

    #[arg(long, env = "ROBONIX_ROBOTDESC_PROVIDER_ID")]
    pub provider_id: Option<String>,

    /// Absolute path to the deployment's robotdesc.yaml. rbnx auto-fills
    /// this from the deployment dir; hand-launching robotdesc requires
    /// providing it explicitly.
    #[arg(long = "robot-yaml", env = "ROBONIX_ROBOTDESC_ROBOT_YAML")]
    pub robot_yaml: Option<PathBuf>,

    #[arg(
        long = "deployment-manifest",
        env = "ROBONIX_ROBOTDESC_DEPLOYMENT_MANIFEST"
    )]
    pub deployment_manifest: Option<PathBuf>,

    /// Optional YAML file whose keys override the built-in defaults
    /// (below CLI/env in the precedence chain). No file ships with
    /// robotdesc; supply one only if the operator needs to pin flags for
    /// hand-launch scenarios.
    #[arg(long, env = "ROBONIX_CONFIG_PATH")]
    pub config: Option<PathBuf>,

    /// Log level for this component (`debug`/`info`/`warn`/`error`). Sets the
    /// log-file level floor; falls back to `ROBONIX_LOG_FILE_LEVEL` / `info`.
    /// Normally arrives inside `--config-json`, not as a standalone flag.
    #[arg(long)]
    pub log: Option<String>,

    /// The component's `system.robotdesc` manifest block, serialised to JSON
    /// by rbnx and passed as one arg (`--config-json '{…}'`). Parsed
    /// by the binary itself — see `robonix_log::init_from_config`,
    /// which reads the `log` key from it so the manifest's
    /// per-component level reaches the log.
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
    provider_id: Option<String>,
    #[serde(default)]
    robot_yaml: Option<PathBuf>,
    #[serde(default)]
    deployment_manifest: Option<PathBuf>,
}

impl RobotdescConfig {
    /// Resolve final config from defaults, optional config file, env, and CLI args.
    pub fn resolve(args: Args) -> Result<Self> {
        let file_cfg = match &args.config {
            Some(path) => load_yaml(path)?,
            None => FileConfig::default(),
        };
        let config_dir = args.config.as_deref().and_then(Path::parent);

        let robot_yaml = match args.robot_yaml.or(file_cfg.robot_yaml) {
            Some(path) => absolute_path(path, config_dir)?,
            None => bail!(
                "missing robot_yaml: set --robot-yaml, ROBONIX_ROBOTDESC_ROBOT_YAML, \
                 or provide it in --config <yaml>"
            ),
        };
        let deployment_manifest = args
            .deployment_manifest
            .or(file_cfg.deployment_manifest)
            .map(|path| absolute_path(path, config_dir))
            .transpose()?
            .unwrap_or_else(|| {
                robot_yaml
                    .parent()
                    .expect("robot_yaml is absolute")
                    .join("robonix_manifest.yaml")
            });

        Ok(Self {
            src_endpoint: args
                .src
                .or(file_cfg.src_endpoint)
                .unwrap_or_else(|| DEFAULT_SRC_ENDPOINT.to_string()),
            listen: args
                .listen
                .or(file_cfg.listen)
                .unwrap_or_else(|| DEFAULT_LISTEN.to_string()),
            provider_id: args
                .provider_id
                .or(file_cfg.provider_id)
                .unwrap_or_else(|| DEFAULT_PROVIDER_ID.to_string()),
            robot_yaml,
            deployment_manifest,
        })
    }

    /// The deployment directory robotdesc reads its manifest from: the
    /// directory holding `robot_yaml` (which must sit alongside
    /// `robonix_manifest.yaml`).
    pub fn manifest_dir(&self) -> &Path {
        self.robot_yaml
            .parent()
            .expect("robot_yaml is an absolute file path")
    }

    pub fn deployment_manifest(&self) -> &Path {
        &self.deployment_manifest
    }
}

fn load_yaml(path: &Path) -> Result<FileConfig> {
    let raw = std::fs::read_to_string(path)
        .with_context(|| format!("read Robotdesc config '{}'", path.display()))?;
    serde_yaml::from_str(&raw)
        .with_context(|| format!("parse Robotdesc config '{}'", path.display()))
}

/// Resolve `path` to an absolute syntactic path. Relative paths anchor
/// on `relative_base` when provided (typically the config-file
/// directory), otherwise on the process cwd. The path need not exist
/// on disk.
fn absolute_path(path: PathBuf, relative_base: Option<&Path>) -> Result<PathBuf> {
    if path.is_absolute() {
        Ok(normalize_path(path))
    } else {
        let base = match relative_base {
            Some(base) if base.is_absolute() => base.to_path_buf(),
            Some(base) => std::env::current_dir()
                .context("resolve current directory")?
                .join(base),
            None => std::env::current_dir().context("resolve current directory")?,
        };
        Ok(normalize_path(base.join(path)))
    }
}

/// Normalize path syntax without touching the filesystem; a lone `.` remains `.`.
fn normalize_path(path: PathBuf) -> PathBuf {
    let mut out = PathBuf::new();
    for component in path.components() {
        match component {
            Component::CurDir => {}
            Component::ParentDir => {
                out.pop();
            }
            other => out.push(other.as_os_str()),
        }
    }
    if out.as_os_str().is_empty() {
        out.push(".");
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    fn repo_root() -> PathBuf {
        normalize_path(PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../.."))
    }

    #[test]
    fn resolves_cli_robot_yaml() {
        let yaml = repo_root().join("examples/test_ci/robotdesc.yaml");
        let args = Args {
            src: None,
            listen: None,
            provider_id: None,
            robot_yaml: Some(yaml.clone()),
            deployment_manifest: None,
            config: None,
            log: None,
            config_json: None,
        };
        let cfg = RobotdescConfig::resolve(args).expect("resolve config");
        assert_eq!(cfg.robot_yaml, yaml);
        assert_eq!(cfg.manifest_dir(), repo_root().join("examples/test_ci"));
        assert_eq!(
            cfg.deployment_manifest(),
            repo_root().join("examples/test_ci/robonix_manifest.yaml")
        );
        assert_eq!(cfg.src_endpoint, DEFAULT_SRC_ENDPOINT);
        assert_eq!(cfg.listen, DEFAULT_LISTEN);
        assert_eq!(cfg.provider_id, DEFAULT_PROVIDER_ID);
    }

    #[test]
    fn accepts_explicit_deployment_manifest() {
        let yaml = repo_root().join("examples/test_ci/robotdesc.yaml");
        let selected = repo_root().join("examples/test_ci/robonix_manifest.profile.yaml");
        let args = Args {
            src: None,
            listen: None,
            provider_id: None,
            robot_yaml: Some(yaml),
            deployment_manifest: Some(selected.clone()),
            config: None,
            log: None,
            config_json: None,
        };
        let cfg = RobotdescConfig::resolve(args).expect("resolve config");
        assert_eq!(cfg.deployment_manifest(), selected);
    }

    #[test]
    fn config_file_relative_robot_yaml_resolves_from_config_dir() {
        let tmp = tempfile::tempdir().expect("tempdir");
        let config_dir = tmp.path().join("deploy");
        std::fs::create_dir_all(&config_dir).expect("create config dir");
        let config_path = config_dir.join("robotdesc.local.yaml");
        std::fs::write(&config_path, "robot_yaml: robotdesc.yaml\n").expect("write config");
        let args = Args {
            src: None,
            listen: None,
            provider_id: None,
            robot_yaml: None,
            deployment_manifest: None,
            config: Some(config_path),
            log: None,
            config_json: None,
        };
        let cfg = RobotdescConfig::resolve(args).expect("resolve config");
        assert_eq!(cfg.robot_yaml, config_dir.join("robotdesc.yaml"));
    }

    #[test]
    fn cli_overrides_config_file() {
        let tmp = tempfile::tempdir().expect("tempdir");
        let config_path = tmp.path().join("robotdesc.local.yaml");
        std::fs::write(
            &config_path,
            "listen: 127.0.0.1:60000\nrobot_yaml: /tmp/file.yaml\n",
        )
        .expect("write config");
        let args = Args {
            src: None,
            listen: Some("127.0.0.1:70000".into()),
            provider_id: None,
            robot_yaml: Some(PathBuf::from("/abs/cli.yaml")),
            deployment_manifest: None,
            config: Some(config_path),
            log: None,
            config_json: None,
        };
        let cfg = RobotdescConfig::resolve(args).expect("resolve config");
        assert_eq!(cfg.listen, "127.0.0.1:70000");
        assert_eq!(cfg.robot_yaml, PathBuf::from("/abs/cli.yaml"));
    }

    #[test]
    fn missing_robot_yaml_is_a_hard_error() {
        let args = Args {
            src: None,
            listen: None,
            provider_id: None,
            robot_yaml: None,
            deployment_manifest: None,
            config: None,
            log: None,
            config_json: None,
        };
        let err = RobotdescConfig::resolve(args).expect_err("no robot_yaml, must fail");
        let msg = format!("{err:#}");
        assert!(msg.contains("robot_yaml"), "{msg}");
    }
}
