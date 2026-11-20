// SPDX-License-Identifier: MulanPSL-2.0
//! Greeting state that a face-greeting deployment shares between processes.
//!
//! The deployment owns a JSON file named by `ROBONIX_GREETING_STATE_PATH`. The
//! greeting skill writes it, navigation reads it to decide whether the robot may
//! move, and Pilot reads it here so a greeting cannot be answered by a stale
//! task. Without the variable every gate below is inert and the robot behaves as
//! it did before, so the state only matters where a deployment opts in.

use crate::pb::pilot::Task;
use robonix_scribe::warn;
use serde_json::{Value, json};
use std::collections::HashSet;
use std::sync::{Mutex, OnceLock};

/// Path of the state file, or `None` when this deployment does not greet.
fn state_path() -> Option<String> {
    let path = std::env::var("ROBONIX_GREETING_STATE_PATH").ok()?;
    let path = path.trim();
    (!path.is_empty()).then(|| path.to_string())
}

/// Report each distinct read failure once.
///
/// The state is read every round, so an unreadable file would otherwise repeat
/// the same line forever. An operator still sees the cause of a held robot.
fn report_once(reason: String) {
    static SEEN: OnceLock<Mutex<HashSet<String>>> = OnceLock::new();
    let seen = SEEN.get_or_init(|| Mutex::new(HashSet::new()));
    let first = seen
        .lock()
        .map(|mut seen| seen.insert(reason.clone()))
        .unwrap_or(true);
    if first {
        warn!("[pilot/greeting] {reason}");
    }
}

/// State used when the file exists but cannot be read or parsed.
///
/// It forbids motion: a greeting whose state is unknown may already own the
/// floor, so the safe reading is "do not move".
fn unreadable(detail: &str) -> Value {
    json!({"movement_allowed": false, "detail": detail})
}

/// Read the state file, returning the path it came from.
///
/// Callers that write beside the file need the path, and re-deriving it from the
/// environment would let a missing variable redirect the write into the process
/// working directory instead of failing.
fn read_state() -> Option<(String, Value)> {
    let path = state_path()?;
    match std::fs::read_to_string(&path) {
        Ok(text) => match serde_json::from_str::<Value>(&text) {
            Ok(value) if value.is_object() => Some((path, value)),
            Ok(_) => {
                report_once(format!("{path} is not a JSON object; holding movement"));
                Some((path, unreadable("greeting state is not an object")))
            }
            Err(error) => {
                report_once(format!(
                    "{path} is not valid JSON ({error}); holding movement"
                ));
                Some((path, unreadable("greeting state is unparseable")))
            }
        },
        Err(error) => {
            if error.kind() != std::io::ErrorKind::NotFound {
                report_once(format!("{path} cannot be read ({error}); holding movement"));
            }
            Some((path, unreadable("greeting state unavailable")))
        }
    }
}

/// Read the current greeting state.
///
/// `None` means no greeting is in progress: the deployment does not greet, so no
/// gate applies. `Some` with `movement_allowed: false` means the deployment
/// greets but its state cannot be read, which has to block motion.
pub(crate) fn greeting_state() -> Option<Value> {
    read_state().map(|(_, state)| state)
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
    let Some((path, state)) = read_state() else {
        return Ok(());
    };
    if state["session_id"] != session_id {
        return Ok(());
    }
    let path = path + ".stop";
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
    let Some((path, state)) = read_state() else {
        return true;
    };
    if state["event_id"] != event_id || state["session_id"] != task.session_id {
        return false;
    }
    match std::fs::read_to_string(path + ".stop") {
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

    #[test]
    fn a_stop_refuses_only_its_own_encounter() {
        let dir = std::env::temp_dir().join(format!("pilot-greeting-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let state = dir.join("greeting_state.json");
        std::fs::write(
            &state,
            json!({"session_id": "s1", "event_id": "e1", "movement_allowed": true}).to_string(),
        )
        .unwrap();
        // SAFETY: no other test in this module reads the variable, and the two
        // that call greeting_task_allowed either return before it is read or
        // compare against a state this test cannot change.
        unsafe { std::env::set_var("ROBONIX_GREETING_STATE_PATH", &state) };

        let current = task("s1", r#"{"greeting_event_id":"e1"}"#);
        let other = task("s1", r#"{"greeting_event_id":"e2"}"#);
        let elsewhere = task("s2", r#"{"greeting_event_id":"e1"}"#);
        assert!(greeting_task_allowed(&current));
        assert!(!greeting_task_allowed(&other));
        assert!(!greeting_task_allowed(&elsewhere));

        stop_greeting("s1").unwrap();
        assert!(
            !greeting_task_allowed(&current),
            "a Stop must refuse the encounter"
        );
        assert!(
            std::fs::read_to_string(dir.join("greeting_state.json.stop")).is_ok(),
            "the marker is written beside the state file"
        );

        unsafe { std::env::remove_var("ROBONIX_GREETING_STATE_PATH") };
        std::fs::remove_dir_all(&dir).unwrap();
    }
}
