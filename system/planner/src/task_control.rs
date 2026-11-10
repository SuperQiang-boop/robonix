// SPDX-License-Identifier: MulanPSL-2.0
//! Harness records for temporarily interrupted work. No new execution RPCs.

use crate::pb::planner::Task;
use crate::pb::planner::{Plan, RtdlNodeState};
use serde_json::{Value, json};
use std::collections::{BTreeMap, HashSet};

#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub(crate) struct TaskControl {
    plans: BTreeMap<String, Value>,
    saved: Vec<Value>,
    completed_calls: HashSet<String>,
    blocked: bool,
    successful_work: bool,
}

impl TaskControl {
    /// Keep exact dispatched steps and original user intent outside model history.
    pub fn dispatch(&mut self, plan: &Plan, goal: &str, criterion: &str) {
        let steps: Vec<Value> = plan
            .nodes
            .iter()
            .filter_map(|node| {
                let call = node.call.as_ref()?;
                Some(json!({"op_id": node.op_id, "description": node.description,
                "provider_id": call.provider_id, "contract_id": call.contract_id,
                "args": serde_json::from_str::<Value>(&call.args_json).unwrap_or(Value::Null),
                "state": 0}))
            })
            .collect();
        self.plans.insert(
            plan.plan_id.clone(),
            json!({"plan_id": plan.plan_id,
            "goal": goal, "success_criterion": criterion, "steps": steps}),
        );
    }

    /// Record actual execution, including results arriving during cancellation.
    pub fn observe(&mut self, plan_id: &str, state: &RtdlNodeState) {
        if state.state == 2
            && state.leaf_result.as_ref().is_some_and(|result| {
                serde_json::from_str::<Value>(&result.output).is_ok_and(|value| {
                    ["ok", "success", "accepted"]
                        .iter()
                        .any(|key| value[*key] == false)
                        || value["exit_code"].as_i64().is_some_and(|code| code != 0)
                        || matches!(
                            value["state"].as_str(),
                            Some("FAILED" | "CANCELED" | "TIMEOUT")
                        )
                })
            })
        {
            self.blocked = true;
        }
        for record in self.plans.get_mut(plan_id).into_iter().chain(
            self.saved
                .iter_mut()
                .filter(|record| record["plan_id"] == plan_id),
        ) {
            if let Some(steps) = record["steps"].as_array_mut() {
                for step in steps.iter_mut().filter(|step| step["op_id"] == state.op_id) {
                    step["state"] = json!(state.state);
                    if let Some(result) = state.leaf_result.as_ref() {
                        step["output"] = json!(result.output);
                        step["error"] = json!(result.error);
                    }
                }
            }
        }
    }

    /// Save only a known, bounded, unfinished plan before cancel is submitted.
    pub fn save(&mut self, plan_id: &str) -> Result<(), String> {
        if self.saved.len() >= 8 {
            return Err(
                "too many nested temporary requests; ask the user which work to retain".into(),
            );
        }
        if self.saved.iter().any(|record| record["plan_id"] == plan_id) {
            return Err("this plan is already retained".into());
        }
        let record = self
            .plans
            .get(plan_id)
            .ok_or("original plan record is unavailable")?;
        self.saved.push(record.clone());
        self.successful_work = false;
        self.blocked = false;
        Ok(())
    }

    /// Failures inhibit automatic return; cancellation alone never counts as work.
    pub fn finish(&mut self, plan_id: &str, failed: bool, canceled: bool, requested: bool) {
        // Executor may report an interrupted shell/SDK leaf as FAILED even
        // when this supervisor's cancellation completed successfully.
        if !requested && (failed || canceled) {
            self.blocked = true;
        } else if !canceled && !requested {
            self.successful_work = true;
        }
        if !failed && !canceled {
            self.saved.retain(|record| record["plan_id"] != plan_id);
        }
        self.plans.remove(plan_id);
    }

    /// Reject automatic continuation after an uncertain cancellation or failure.
    pub fn block(&mut self) {
        self.blocked = true;
    }

    /// A global user stop discards all automatic return arrangements.
    pub fn clear(&mut self) {
        self.saved.clear();
        self.completed_calls.clear();
        self.successful_work = false;
        self.blocked = false;
    }

    /// Replacing one temporary task preserves its unrelated retained parents.
    pub fn replace(&mut self, plan_id: &str) {
        self.saved.retain(|record| record["plan_id"] != plan_id);
        self.completed_calls.clear();
        self.successful_work = false;
    }

    /// Restore intent, never replay a stored physical plan without fresh planning.
    pub fn resume(&mut self, verified: bool) -> Option<Value> {
        if !verified || self.blocked || !self.successful_work {
            return None;
        }
        let record = self.saved.pop()?;
        self.completed_calls.clear();
        for step in record["steps"].as_array().into_iter().flatten() {
            if step["state"] == 2 {
                // Existing Executor SUCCEEDED value.
                self.completed_calls.insert(format!(
                    "{}|{}|{}",
                    step["provider_id"].as_str().unwrap_or_default(),
                    step["contract_id"].as_str().unwrap_or_default(),
                    step["args"]
                ));
            }
        }
        self.successful_work = false;
        Some(record)
    }

    /// Prevent a resumed plan from repeating an already successful exact call.
    pub fn repeats_completed(&self, calls: &HashSet<String>) -> bool {
        calls.iter().any(|call| self.completed_calls.contains(call))
    }

    /// Include retained tasks every round, independently of history compaction.
    pub fn prompt(&self) -> String {
        if self.saved.is_empty() {
            return String::new();
        }
        format!(
            "\nRetained unfinished tasks (last saved returns first): {}\n\
            Automatic return blocked by failure/uncertainty: {}. Complete and verify \
            the temporary request before marking it done. Never restart a failed task \
            or replay completed physical steps. A replacement drops its target; a global stop discards \
            all automatic return.\n",
            json!(self.saved),
            self.blocked
        )
    }

    pub fn has_saved(&self) -> bool {
        !self.saved.is_empty()
    }

    pub fn blocks_actions(&self) -> bool {
        self.blocked && self.has_saved()
    }

    /// Fresh human input permits a new decision, without proving task completion.
    pub fn new_input(&mut self) {
        self.blocked = false;
        self.successful_work = false;
        self.completed_calls.clear();
    }
}

/// Read the opt-in deployment snapshot; unavailable state never permits motion.
pub(crate) fn greeting_state() -> Option<Value> {
    let path = std::env::var("ROBONIX_GREETING_STATE_PATH").ok()?;
    if path.trim().is_empty() {
        return None;
    }
    Some(
        std::fs::read_to_string(path)
            .ok()
            .and_then(|text| serde_json::from_str::<Value>(&text).ok())
            .filter(Value::is_object)
            .unwrap_or_else(
                || json!({"movement_allowed": false, "detail": "greeting state unavailable"}),
            ),
    )
}

pub(crate) fn greeting_blocks(state: &Option<Value>) -> bool {
    state
        .as_ref()
        .is_some_and(|value| value["movement_allowed"] != true)
}

/// A Stop invalidates late voice admission for this exact greeting encounter.
pub(crate) fn stop_greeting(session_id: &str) -> std::io::Result<()> {
    let Some(state) = greeting_state() else {
        return Ok(());
    };
    if state["session_id"] != session_id {
        return Ok(());
    }
    let path = std::env::var("ROBONIX_GREETING_STATE_PATH").unwrap_or_default() + ".stop";
    let temporary = path.clone() + ".tmp";
    std::fs::write(
        &temporary,
        json!({"session_id": session_id, "event_id": state["event_id"]}).to_string(),
    )?;
    std::fs::rename(temporary, path)
}

/// Face identity is context, never a substitute for Gateway voice authorization.
pub(crate) fn greeting_task_allowed(task: &Task) -> bool {
    let context = serde_json::from_str::<Value>(&task.context_json).unwrap_or(Value::Null);
    let Some(event_id) = context["greeting_event_id"].as_str() else {
        return true;
    };
    let Some(state) = greeting_state() else {
        return true;
    };
    if state["event_id"] != event_id || state["session_id"] != task.session_id {
        return false;
    }
    let path = std::env::var("ROBONIX_GREETING_STATE_PATH").unwrap_or_default() + ".stop";
    match std::fs::read_to_string(path) {
        Ok(text) => serde_json::from_str::<Value>(&text).is_ok_and(|stopped| {
            stopped["event_id"] != event_id || stopped["session_id"] != task.session_id
        }),
        Err(error) => error.kind() == std::io::ErrorKind::NotFound,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::pb::planner::{CapabilityCall, RtdlNode};

    /// Check control.
    fn control() -> TaskControl {
        let mut control = TaskControl::default();
        control.dispatch(
            &Plan {
                plan_id: "A".into(),
                nodes: vec![RtdlNode {
                    op_id: "move".into(),
                    call: Some(CapabilityCall {
                        provider_id: "robot".into(),
                        contract_id: "navigate".into(),
                        args_json: "{\"target\":\"A\"}".into(),
                        ..Default::default()
                    }),
                    ..Default::default()
                }],
                ..Default::default()
            },
            "go to A",
            "arrived at A",
        );
        control
    }

    #[test]
    /// Check temporary work returns only after success.
    fn temporary_work_returns_only_after_success() {
        let mut control = control();
        control.save("A").unwrap();
        control.finish("A", false, true, true);
        assert!(control.resume(true).is_none());
        control.finish("B", false, false, false);
        assert!(control.resume(false).is_none());
        assert_eq!(control.resume(true).unwrap()["goal"], "go to A");
        assert!(control.resume(true).is_none());
    }

    /// Nested temporary requests return in order; replacing B does not erase A.
    #[test]
    fn nested_and_replaced_temporary_requests_preserve_the_original() {
        let mut control = control();
        control.save("A").unwrap();
        control.finish("A", false, true, true);
        let mut replacement = control.plans.clone();
        // Model the interrupted B using the same stored shape as an ordinary plan.
        replacement.insert(
            "B".into(),
            json!({"plan_id": "B", "goal": "go to B", "steps": []}),
        );
        control.plans = replacement;
        control.save("B").unwrap();
        control.finish("B", false, true, true);
        control.finish("C", false, false, false);
        assert_eq!(control.resume(true).unwrap()["goal"], "go to B");
        control.replace("B");
        assert!(control.resume(true).is_none());
        control.finish("replacement-B", false, false, false);
        assert_eq!(control.resume(true).unwrap()["goal"], "go to A");
    }

    #[test]
    /// Check expected cancel leaf failure does not fail the temporary request.
    fn expected_cancel_leaf_failure_does_not_fail_the_temporary_request() {
        let mut control = control();
        control.save("A").unwrap();
        control.finish("A", true, true, true);
        assert!(!control.blocks_actions());
        assert!(control.resume(true).is_none());
        control.finish("B", false, false, false);
        assert!(control.resume(false).is_none());
        assert_eq!(control.resume(true).unwrap()["goal"], "go to A");
    }

    #[test]
    /// Check failure stop and unknown cancellation never restart original work.
    fn failure_stop_and_unknown_cancellation_never_restart_original_work() {
        for (failed, canceled) in [(true, false), (false, true)] {
            let mut control = control();
            control.save("A").unwrap();
            control.finish("B", failed, canceled, false);
            assert!(control.resume(true).is_none());
            control.clear();
            control.finish("C", false, false, false);
            assert!(control.resume(true).is_none());
        }
    }

    #[test]
    /// Check transport success with capability error cannot trigger return.
    fn transport_success_with_capability_error_cannot_trigger_return() {
        let mut control = control();
        control.save("A").unwrap();
        control.observe(
            "B",
            &RtdlNodeState {
                state: 2,
                leaf_result: Some(crate::pb::planner::CapabilityCallResult {
                    success: true,
                    output: "{\"ok\":false}".into(),
                    ..Default::default()
                }),
                ..Default::default()
            },
        );
        control.finish("B", false, false, false);
        assert!(control.resume(true).is_none());
        assert!(control.blocks_actions());
    }

    #[test]
    /// Check completed steps are retained and cannot be replayed.
    fn completed_steps_are_retained_and_cannot_be_replayed() {
        let mut control = control();
        control.save("A").unwrap();
        control.observe(
            "A",
            &RtdlNodeState {
                op_id: "move".into(),
                state: 2,
                ..Default::default()
            },
        );
        control.finish("B", false, false, false);
        let saved = control.resume(true).unwrap();
        assert_eq!(saved["steps"][0]["state"], 2);
        assert!(
            control.repeats_completed(&HashSet::from(["robot|navigate|{\"target\":\"A\"}".into()]))
        );
    }

    #[test]
    /// Check missing or blocked greeting does not authorize motion.
    fn missing_or_blocked_greeting_does_not_authorize_motion() {
        assert!(!greeting_blocks(&None));
        assert!(greeting_blocks(&Some(json!({}))));
        assert!(greeting_blocks(&Some(json!({"movement_allowed": false}))));
        assert!(!greeting_blocks(&Some(json!({"movement_allowed": true}))));
    }
}
