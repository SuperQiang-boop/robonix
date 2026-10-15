use crate::pb::pilot::{CapabilityCallResult, Plan};
use anyhow::{Context, Result, bail};
use serde_json::{Value, json};
use std::collections::{HashMap, VecDeque};
use std::time::{Duration, Instant};

const NAVIGATE: &str = "robonix/service/navigation/navigate";
const SCENE_GOALS: [&str; 2] = [
    "robonix/system/scene/goal_near",
    "robonix/system/scene/goal_room",
];
const MAX_AGE: Duration = Duration::from_secs(120);
const MAX_GOALS: usize = 32;
const POSITION_EPSILON: f64 = 0.0001;

struct ScenePose {
    x: f64,
    y: f64,
    yaw: f64,
    received_at: Instant,
}

/// Keep only dispatched, successful Scene resolutions in the current user interaction.
#[derive(Default)]
pub(crate) struct SceneNavigationGoals {
    pending: HashMap<String, (String, String)>,
    poses: VecDeque<ScenePose>,
}

impl SceneNavigationGoals {
    pub(crate) fn clear(&mut self) {
        self.pending.clear();
        self.poses.clear();
    }

    /// Accept results only for exact calls actually dispatched by this interaction.
    pub(crate) fn track_plan(&mut self, plan: &Plan) {
        for call in plan.nodes.iter().filter_map(|node| node.call.as_ref()) {
            if SCENE_GOALS.contains(&call.contract_id.as_str()) {
                self.pending.insert(
                    call.call_id.clone(),
                    (call.provider_id.clone(), call.contract_id.clone()),
                );
            }
        }
    }

    /// Cache finite map-frame goals only after a matching dispatched, reachable Scene result.
    pub(crate) fn observe(&mut self, result: &CapabilityCallResult, now: Instant) {
        let Some((provider, contract)) = self.pending.get(&result.call_id) else {
            return;
        };
        if provider != &result.provider_id || contract != &result.contract_id {
            return;
        }
        self.pending.remove(&result.call_id);
        if !result.success {
            return;
        }
        let Ok(output) = serde_json::from_str::<Value>(&result.output) else {
            return;
        };
        if output.get("reachable").and_then(Value::as_bool) != Some(true) {
            return;
        }
        let coordinates = ["x", "y", "yaw"].map(|key| output.get(key).and_then(Value::as_f64));
        let [Some(x), Some(y), Some(yaw)] = coordinates else {
            return;
        };
        if ![x, y, yaw].iter().all(|value| value.is_finite()) {
            return;
        }
        self.poses.retain(|pose| !same_position(pose, x, y));
        self.poses.push_back(ScenePose {
            x,
            y,
            yaw,
            received_at: now,
        });
        while self.poses.len() > MAX_GOALS {
            self.poses.pop_front();
        }
    }

    /// Replace LLM quaternion arithmetic only for matching, fresh Scene-resolved map goals.
    /// Raw goals in another frame or at other coordinates remain untouched.
    pub(crate) fn canonicalize(&self, plan: &mut Plan, now: Instant) -> Result<usize> {
        let has_resolution = plan
            .nodes
            .iter()
            .filter_map(|node| node.call.as_ref())
            .any(|call| SCENE_GOALS.contains(&call.contract_id.as_str()));
        let mut corrected = 0;
        for call in plan.nodes.iter_mut().filter_map(|node| node.call.as_mut()) {
            if call.contract_id != NAVIGATE {
                continue;
            }
            if has_resolution {
                bail!("Scene goal resolution and navigation must be in separate planning rounds");
            }
            let mut args: Value = serde_json::from_str(&call.args_json)
                .context("navigation args must be valid JSON")?;
            let frame = args
                .pointer("/goal/header/frame_id")
                .and_then(Value::as_str);
            if !matches!(frame, Some("map") | Some("")) {
                continue;
            }
            let coordinates = ["/goal/pose/position/x", "/goal/pose/position/y"]
                .map(|pointer| args.pointer(pointer).and_then(Value::as_f64));
            let [Some(x), Some(y)] = coordinates else {
                continue;
            };
            let Some(pose) = self
                .poses
                .iter()
                .rev()
                .find(|pose| same_position(pose, x, y))
            else {
                continue;
            };
            if now.saturating_duration_since(pose.received_at) > MAX_AGE {
                bail!(
                    "Scene navigation goal is older than 120 seconds; resolve it again before navigating"
                );
            }
            let orientation = json!({
                "x": 0.0, "y": 0.0,
                "z": (pose.yaw / 2.0).sin(), "w": (pose.yaw / 2.0).cos(),
            });
            let target = args
                .pointer_mut("/goal/pose")
                .and_then(Value::as_object_mut)
                .context("Scene-derived navigation goal must contain an object pose")?;
            if !target
                .get("orientation")
                .is_some_and(|current| same_orientation(current, &orientation))
            {
                target.insert("orientation".to_string(), orientation);
                call.args_json = serde_json::to_string(&args)?;
                corrected += 1;
            }
        }
        Ok(corrected)
    }
}

fn same_position(pose: &ScenePose, x: f64, y: f64) -> bool {
    (pose.x - x).abs() <= POSITION_EPSILON && (pose.y - y).abs() <= POSITION_EPSILON
}

/// Ignore JSON floating-point round-trip noise, not a meaningful heading discrepancy.
fn same_orientation(current: &Value, expected: &Value) -> bool {
    ["x", "y", "z", "w"].iter().all(|axis| {
        match (current[*axis].as_f64(), expected[*axis].as_f64()) {
            (Some(current_value), Some(expected_value)) => {
                (current_value - expected_value).abs() <= 1e-12
            }
            _ => false,
        }
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::pb::pilot::{CapabilityCall, RtdlNode};

    /// Build a single dispatched capability call without Atlas or physical execution.
    fn plan(contract: &str, args: Value) -> Plan {
        Plan {
            nodes: vec![RtdlNode {
                node_kind: 2,
                call: Some(CapabilityCall {
                    call_id: "1:0".into(),
                    provider_id: "scene".into(),
                    contract_id: contract.into(),
                    args_json: args.to_string(),
                }),
                ..Default::default()
            }],
            ..Default::default()
        }
    }

    /// Reproduce the exact chair resolution whose yaw was mistranscribed by Pilot.
    fn resolution(reachable: bool) -> CapabilityCallResult {
        CapabilityCallResult {
            call_id: "1:0".into(),
            provider_id: "scene".into(),
            contract_id: SCENE_GOALS[0].into(),
            success: true,
            output: json!({"reachable": reachable, "x": 0.5431279931217432,
                "y": 3.416287550702691, "yaw": -2.392551870225323})
            .to_string(),
            ..Default::default()
        }
    }

    /// Reproduce the recorded incorrect quaternion while retaining the correct XY.
    fn navigation() -> Plan {
        plan(
            NAVIGATE,
            json!({"goal": {"header": {"frame_id": "map"}, "pose": {
                "position": {"x": 0.5431279931217432, "y": 3.416287550702691, "z": 0.0},
                "orientation": {"x": 0.0, "y": 0.0, "z": -0.6816, "w": 0.7317}
            }}}),
        )
    }

    /// Admit the fixture through the same provenance path as production results.
    fn cache(now: Instant) -> SceneNavigationGoals {
        let mut goals = SceneNavigationGoals::default();
        goals.track_plan(&plan(
            SCENE_GOALS[0],
            json!({"object_id": "scene.object.chair_002"}),
        ));
        goals.observe(&resolution(true), now);
        goals
    }

    #[test]
    /// Preserve Scene's yaw exactly and leave positions and repeated canonicalization stable.
    fn recorded_chair_yaw_is_converted_before_dispatch_without_changing_position() {
        let now = Instant::now();
        let goals = cache(now);
        let mut plan = navigation();
        let before: Value =
            serde_json::from_str(&plan.nodes[0].call.as_ref().unwrap().args_json).unwrap();
        assert_eq!(goals.canonicalize(&mut plan, now).unwrap(), 1);
        let args: Value =
            serde_json::from_str(&plan.nodes[0].call.as_ref().unwrap().args_json).unwrap();
        let quaternion = &args["goal"]["pose"]["orientation"];
        let yaw = 2.0
            * quaternion["z"]
                .as_f64()
                .unwrap()
                .atan2(quaternion["w"].as_f64().unwrap());
        assert!((yaw - -2.392551870225323).abs() < 1e-12);
        assert_eq!(
            args["goal"]["pose"]["position"],
            before["goal"]["pose"]["position"]
        );
        assert_eq!(goals.canonicalize(&mut plan, now).unwrap(), 0);
    }

    #[test]
    /// Do not alter independent goals, coordinate frames or unreachable Scene responses.
    fn raw_positions_other_frames_and_failed_resolutions_are_not_rewritten() {
        let now = Instant::now();
        for (pointer, value) in [
            ("/goal/header/frame_id", json!("odom")),
            ("/goal/pose/position/x", json!(8.0)),
        ] {
            let goals = cache(now);
            let mut plan = navigation();
            let call = plan.nodes[0].call.as_mut().unwrap();
            let mut args: Value = serde_json::from_str(&call.args_json).unwrap();
            *args.pointer_mut(pointer).unwrap() = value;
            call.args_json = args.to_string();
            let original = call.args_json.clone();
            assert_eq!(goals.canonicalize(&mut plan, now).unwrap(), 0);
            assert_eq!(plan.nodes[0].call.as_ref().unwrap().args_json, original);
        }
        let mut goals = SceneNavigationGoals::default();
        goals.track_plan(&plan(SCENE_GOALS[0], json!({})));
        goals.observe(&resolution(false), now);
        assert_eq!(goals.canonicalize(&mut navigation(), now).unwrap(), 0);
    }

    #[test]
    /// Require fresh resolution after expiration and discard results from before a user steer.
    fn stale_goals_require_resolution_and_user_steers_clear_pending_provenance() {
        let now = Instant::now();
        let mut goals = cache(now);
        assert!(
            goals
                .canonicalize(&mut navigation(), now + MAX_AGE + Duration::from_secs(1))
                .is_err()
        );
        goals.track_plan(&plan(SCENE_GOALS[0], json!({})));
        goals.clear();
        goals.observe(&resolution(true), now);
        assert_eq!(goals.canonicalize(&mut navigation(), now).unwrap(), 0);
    }

    #[test]
    /// A mismatched provider must not be able to rewrite subsequent navigation orientation.
    fn unrelated_provider_or_contract_cannot_supply_a_scene_goal() {
        let now = Instant::now();
        let mut goals = SceneNavigationGoals::default();
        goals.track_plan(&plan(SCENE_GOALS[0], json!({})));
        let mut result = resolution(true);
        result.provider_id = "unrelated".into();
        goals.observe(&result, now);
        assert_eq!(goals.canonicalize(&mut navigation(), now).unwrap(), 0);
        goals.observe(&resolution(true), now);
        assert_eq!(goals.canonicalize(&mut navigation(), now).unwrap(), 1);
    }

    #[test]
    /// Both Scene goal contracts produce unit planar quaternions across heading quadrants.
    fn room_and_object_headings_are_converted_without_trigonometric_approximation() {
        let now = Instant::now();
        for contract in SCENE_GOALS {
            for yaw in [-3.0_f64, -1.5, 0.0, 1.5, 3.0] {
                let mut goals = SceneNavigationGoals::default();
                goals.track_plan(&plan(contract, json!({})));
                let mut result = resolution(true);
                result.contract_id = contract.into();
                let mut output: Value = serde_json::from_str(&result.output).unwrap();
                output["yaw"] = json!(yaw);
                result.output = output.to_string();
                goals.observe(&result, now);
                let mut target = navigation();
                assert_eq!(goals.canonicalize(&mut target, now).unwrap(), 1);
                let args: Value =
                    serde_json::from_str(&target.nodes[0].call.as_ref().unwrap().args_json)
                        .unwrap();
                let quaternion = &args["goal"]["pose"]["orientation"];
                let rotation_z = quaternion["z"].as_f64().unwrap();
                let rotation_w = quaternion["w"].as_f64().unwrap();
                assert!((rotation_z * rotation_z + rotation_w * rotation_w - 1.0).abs() < 1e-12);
                assert!((2.0 * rotation_z.atan2(rotation_w) - yaw).abs() < 1e-12);
            }
        }
    }

    #[test]
    /// Transport failures, missing yaw and malformed outputs never become authoritative goals.
    fn invalid_or_failed_scene_results_are_not_admitted() {
        let now = Instant::now();
        for output in [
            "invalid JSON",
            "{}",
            r#"{"reachable":true,"x":0.5431279931217432,"y":3.416287550702691,"yaw":null}"#,
        ] {
            let mut goals = SceneNavigationGoals::default();
            goals.track_plan(&plan(SCENE_GOALS[0], json!({})));
            let mut result = resolution(true);
            result.output = output.into();
            goals.observe(&result, now);
            assert_eq!(goals.canonicalize(&mut navigation(), now).unwrap(), 0);
        }
        let mut goals = SceneNavigationGoals::default();
        goals.track_plan(&plan(SCENE_GOALS[0], json!({})));
        let mut result = resolution(true);
        result.success = false;
        goals.observe(&result, now);
        assert_eq!(goals.canonicalize(&mut navigation(), now).unwrap(), 0);
    }

    #[test]
    /// A dependent navigation cannot run before its Scene result has arrived.
    fn dependent_scene_and_navigation_calls_cannot_share_one_tree() {
        let goals = SceneNavigationGoals::default();
        let mut mixed = navigation();
        mixed.nodes.extend(plan(SCENE_GOALS[0], json!({})).nodes);
        assert!(goals.canonicalize(&mut mixed, Instant::now()).is_err());
    }
}
