// SPDX-License-Identifier: MulanPSL-2.0
//! Greeting state that a face-greeting deployment shares between processes.
//!
//! The deployment owns a JSON file named by `ROBONIX_GREETING_STATE_PATH`. The
//! greeting skill writes it, navigation reads it to decide whether the robot may
//! move, and Pilot reads it here so a greeting cannot be answered by a stale
//! task. Without the variable every gate below is inert and the robot behaves as
//! it did before, so the state only matters where a deployment opts in.

use crate::pb::pilot::Task;
use serde_json::{Value, json};

/// Path of the state file, or `None` when this deployment does not greet.
fn state_path() -> Option<String> {
    let path = std::env::var("ROBONIX_GREETING_STATE_PATH").ok()?;
    let path = path.trim();
    (!path.is_empty()).then(|| path.to_string())
}

/// Read the current greeting state.
///
/// `None` means no greeting is in progress: the deployment does not greet, so no
/// gate applies. `Some` with `movement_allowed: false` means the deployment
/// greets but the state cannot be read, which has to block motion — a greeting
/// whose state is unknown could already own the floor.
pub(crate) fn greeting_state() -> Option<Value> {
    let path = state_path()?;
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

/// Whether this state forbids the robot from moving.
pub(crate) fn greeting_blocks(state: &Option<Value>) -> bool {
    state
        .as_ref()
        .is_some_and(|value| value["movement_allowed"] != true)
}

/// Invalidate late voice admission for one greeting encounter after a Stop.
///
/// The marker is written beside the state file rather than into it, so the skill
/// that owns the state does not have to observe Pilot's write. A Stop for a
/// different session leaves the current encounter untouched.
pub(crate) fn stop_greeting(session_id: &str) -> std::io::Result<()> {
    let Some(state) = greeting_state() else {
        return Ok(());
    };
    if state["session_id"] != session_id {
        return Ok(());
    }
    let path = state_path().unwrap_or_default() + ".stop";
    let temporary = path.clone() + ".tmp";
    std::fs::write(
        &temporary,
        json!({"session_id": session_id, "event_id": state["event_id"]}).to_string(),
    )?;
    std::fs::rename(temporary, path)
}

/// Whether a task may still be admitted for the greeting it belongs to.
///
/// A task that names no greeting event is ordinary work and always allowed. A
/// task that names one is allowed only while that encounter is current, and only
/// until a Stop for it was recorded. A face match is context, never a substitute
/// for the voice authorization Liaison attached to the task.
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
    let path = state_path().unwrap_or_default() + ".stop";
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

    /// A task carrying the greeting event and session the state file names.
    fn task(session_id: &str, context: &str) -> Task {
        Task {
            session_id: session_id.into(),
            context_json: context.into(),
            ..Default::default()
        }
    }

    #[test]
    fn unreadable_state_blocks_motion() {
        assert!(!greeting_blocks(&None));
        assert!(greeting_blocks(&Some(json!({}))));
        assert!(greeting_blocks(&Some(json!({"movement_allowed": false}))));
        assert!(!greeting_blocks(&Some(json!({"movement_allowed": true}))));
    }

    #[test]
    fn a_task_without_a_greeting_event_is_always_allowed() {
        assert!(greeting_task_allowed(&task("s1", "")));
        assert!(greeting_task_allowed(&task(
            "s1",
            r#"{"modality":"voice"}"#
        )));
    }
}
