// SPDX-License-Identifier: MulanPSL-2.0
//
// Planner-side native Robotdesc awareness.
//
// Robotdesc exposes robot body data as gRPC contracts. Planner fetches the
// default robot's Robotdesc YAML once at startup, then injects it into every
// turn's system prompt so the model knows its body without the user first
// calling a bridge tool.
//
// Deliberately YAML only. The URDF is a full kinematic XML tree whose link
// and joint geometry the planner never reasons over, so injecting it only
// spent context and gave the model a second, lower-level body description to
// contradict robotdesc.yaml with. Robotdesc still serves get_urdf for consumers that
// need the kinematics; it just does not belong in a prompt.

use crate::pb::contracts::{
    robonix_system_robotdesc_get_health_client::RobonixSystemRobotdescGetHealthClient,
    robonix_system_robotdesc_get_yaml_client::RobonixSystemRobotdescGetYamlClient,
};
use crate::pb::robotdesc::{GetHealthRequest, GetYamlRequest};
use anyhow::{Context, Result};
use robonix_log::warn;
use robonix_src::client::{self as src_client, SrcClient};

const GET_YAML_CONTRACT: &str = "robonix/system/robotdesc/get_yaml";
const GET_HEALTH_CONTRACT: &str = "robonix/system/robotdesc/get_health";

pub async fn fetch_runtime_prompt_block(src: &mut SrcClient, consumer_id: &str) -> String {
    match fetch_health(src, consumer_id).await {
        Ok(value) => format!(
            "\n\n## Current embodiment state (from Robotdesc)\n\
             This snapshot was refreshed immediately before planning. Fresh fields are \
             authoritative. Stale or missing fields mean unknown; never reconstruct them \
             from conversation history. `likely_holding` means the calibrated gripper is \
             not fully open; it does not identify the object.\n\n{}\n",
            serde_json::to_string(&value).unwrap_or_else(|_| "{}".into())
        ),
        Err(error) => format!(
            "\n\n## Current embodiment state (from Robotdesc)\n\
             {{\"available\":false,\"error\":{}}}\n",
            serde_json::to_string(&error.to_string()).unwrap_or_else(|_| "\"unknown\"".into())
        ),
    }
}

async fn fetch_health(src: &mut SrcClient, consumer_id: &str) -> Result<serde_json::Value> {
    let (channel_id, _provider_id, channel) =
        src_client::connect_to_capability(src, consumer_id, GET_HEALTH_CONTRACT)
            .await
            .context("connect to Robotdesc get_health")?;
    let result = async {
        let response = RobonixSystemRobotdescGetHealthClient::new(channel)
            .get_health(GetHealthRequest {})
            .await
            .context("call Robotdesc get_health")?
            .into_inner();
        let snapshot = response
            .snapshot
            .context("Robotdesc has not published a health snapshot yet")?;
        let components: Vec<_> = snapshot
            .components
            .into_iter()
            .map(|component| {
                serde_json::json!({
                    "id": component.id,
                    "parent_id": component.parent_id,
                    "kind": component.kind,
                    "health": component.health,
                    "operational_state": component.operational_state,
                    "online": component.online,
                    "detail": component.detail,
                })
            })
            .collect();
        let actuators: Vec<_> = snapshot
            .actuators
            .into_iter()
            .map(|actuator| {
                serde_json::json!({
                    "component_id": actuator.component_id,
                    "joint_name": actuator.joint_name,
                    "position": actuator.position.map(|value| serde_json::json!({
                        "value": value.value, "unit": value.unit, "quality": value.quality,
                    })),
                    "communication_ok": actuator.communication_ok,
                })
            })
            .collect();
        let metrics: Vec<_> = snapshot
            .metrics
            .into_iter()
            .map(|metric| {
                serde_json::json!({
                    "component_id": metric.component_id,
                    "name": metric.name,
                    "value": metric.value.map(|value| serde_json::json!({
                        "value": value.value, "unit": value.unit, "quality": value.quality,
                    })),
                })
            })
            .collect();
        Ok::<_, anyhow::Error>(serde_json::json!({
            "available": true,
            "body_id": snapshot.body_id,
            "seq": snapshot.seq,
            "source_ts_ns": snapshot.source_ts_ns,
            "ttl_ms": snapshot.ttl_ms,
            "components": components,
            "actuators": actuators,
            "metrics": metrics,
            "safety": snapshot.safety.map(|safety| serde_json::json!({
                "motion_allowed": safety.motion_allowed,
                "motor_power_allowed": safety.motor_power_allowed,
                "aggregate_state": safety.aggregate_state,
                "detail": safety.detail,
            })),
        }))
    }
    .await;
    let _ = src.disconnect_capability(&channel_id).await;
    result
}

pub async fn fetch_system_prompt_block(
    src: &mut SrcClient,
    consumer_id: &str,
) -> Result<Option<String>> {
    let yaml = match fetch_yaml(src, consumer_id).await {
        Ok(text) => text,
        Err(e) => {
            warn!(
                "[planner/robotdesc] get_yaml unavailable; continuing without Robotdesc context: {e:#}"
            );
            return Ok(None);
        }
    };
    let mut block = String::from(
        "\n\n## Robot Body Context (from Robotdesc)\n\n\
         This is the robot's self-description, loaded automatically at Planner startup. \
         Treat it as authoritative HARD CONSTRAINTS for the robot's body, sensors, \
         frames, limits, and deployment-specific notes. Do not ask the user to call \
         Robotdesc manually unless this context is absent or stale.\n\n\
         ### Hard planning rules from Robotdesc\n\n\
         - Sensor placement and modality in `robotdesc.yaml` are binding. Do not invent \
         sensors, viewpoints, arms, grippers, or degrees of freedom that are not listed.\n\
         - Before planning an observation, match the user's requested viewpoint \
         (front / rear / left / right / top, etc.) against the listed sensors' \
         `placement`, `human_label`, and `cannot_do` notes.\n\
         - If the requested viewpoint is not directly available from the sensors \
         listed in Robotdesc, say so explicitly. Do NOT call a camera with one placement \
         and describe its image as if it came from a different placement.\n\
         - If a viewpoint can be achieved only by moving the base (for example, \
         rotate 180 degrees, then use the front camera), state that plan clearly \
         and use motion + observation capabilities rather than pretending a missing \
         sensor exists.\n\n\
         ### robotdesc.yaml (compact JSON)\n\n```json\n",
    );
    block.push_str(&compact_yaml(&yaml));
    block.push_str("\n```\n");
    Ok(Some(block))
}

/// Serialize Robotdesc YAML without comments or presentation whitespace while
/// preserving every data field the planner can act on.
fn compact_yaml(raw: &str) -> String {
    match serde_yaml::from_str::<serde_yaml::Value>(raw) {
        Ok(value) => serde_json::to_string(&value).unwrap_or_else(|_| raw.trim().to_string()),
        Err(error) => {
            warn!(
                "[planner/robotdesc] could not compact robotdesc.yaml; keeping source text: {error}"
            );
            raw.trim().to_string()
        }
    }
}

async fn fetch_yaml(src: &mut SrcClient, consumer_id: &str) -> Result<String> {
    let (channel_id, _provider_id, channel) =
        src_client::connect_to_capability(src, consumer_id, GET_YAML_CONTRACT)
            .await
            .context("connect to Robotdesc get_yaml")?;
    let result = async {
        let mut client = RobonixSystemRobotdescGetYamlClient::new(channel);
        let response = client
            .get_yaml(GetYamlRequest {
                robot_id: String::new(),
            })
            .await
            .context("call Robotdesc get_yaml")?
            .into_inner();
        Ok::<_, anyhow::Error>(response.yaml_text)
    }
    .await;
    let _ = src.disconnect_capability(&channel_id).await;
    result
}

#[cfg(test)]
mod tests {
    use super::compact_yaml;

    #[test]
    fn representative_robotdesc_context_is_smaller_without_dropping_body_facts() {
        let yaml = include_str!("../../../examples/webots/robotdesc.yaml");
        let compact_yaml = compact_yaml(yaml);
        let yaml_value: serde_yaml::Value = serde_yaml::from_str(yaml).unwrap();
        let compact_value: serde_json::Value = serde_json::from_str(&compact_yaml).unwrap();
        let before = yaml.len();
        let after = compact_yaml.len();
        eprintln!(
            "representative Robotdesc prompt bytes: before={before} after={after} reduction={:.1}%",
            100.0 * (before - after) as f64 / before as f64
        );
        assert!(after < before);
        assert_eq!(serde_json::to_value(yaml_value).unwrap(), compact_value);
        assert!(compact_yaml.contains("front"));
    }
}
