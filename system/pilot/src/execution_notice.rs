// SPDX-License-Identifier: MulanPSL-2.0
//! Deterministic execution notices built from Executor events, not model prose.

use crate::pb::pilot::{RtdlNodeState, SessionStatusEvent};
use crate::service::SessionState;

/// Announce the first real RUNNING event for one plan, before a final result.
pub fn running(session: &str, plan: &str, description: &str) -> SessionStatusEvent {
    SessionStatusEvent {
        session_id: session.into(),
        state: SessionState::Active as u32,
        message: format!("[plan {plan}] 执行中（RUNNING）：{description}"),
    }
}

/// Preserve timeout/cancel outcomes even when a sequence parent reports FAILED.
/// Genuine failed leaves take priority in mixed-result parallel executions.
/// The notice only claims execution completion, never unverified side effects.
pub fn terminal(
    session: &str,
    plan: &str,
    description: &str,
    results: &[RtdlNodeState],
    any_failed: bool,
    canceled: bool,
) -> SessionStatusEvent {
    let failed_leaf = results
        .iter()
        .any(|r| r.state == 3 && r.leaf_result.is_some());
    let (state, label) = if failed_leaf {
        (3, "执行失败（FAILED）")
    } else if results.iter().any(|r| r.state == 5) {
        (5, "执行超时（TIMEOUT）")
    } else if canceled || results.iter().any(|r| r.state == 4) {
        (4, "已取消（CANCELED）")
    } else if any_failed || results.is_empty() || results.iter().any(|r| r.state != 2) {
        (3, "执行失败（FAILED）")
    } else {
        (2, "执行成功（SUCCEEDED）")
    };
    let detail = results
        .iter()
        .filter(|r| r.state == state || (state != 2 && r.state != 2))
        .find_map(|r| {
            let detail = r
                .leaf_result
                .as_ref()
                .map(|leaf| leaf.error.trim())
                .filter(|text| !text.is_empty())
                .unwrap_or(r.operator_detail.trim());
            (!detail.is_empty()).then_some(detail)
        })
        .unwrap_or(if results.is_empty() {
            "Executor 未返回完整结果"
        } else {
            ""
        });
    let suffix = if detail.is_empty() {
        String::new()
    } else {
        format!("；{}", detail.chars().take(400).collect::<String>())
    };
    SessionStatusEvent {
        session_id: session.into(),
        state: if state == 2 {
            SessionState::Completed as u32
        } else {
            SessionState::Failed as u32
        },
        message: format!("[plan {plan}] {label}：{description}{suffix}"),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::pb::pilot::CapabilityCallResult;

    /// Construct a real leaf result alongside the sequence's aggregate state.
    fn outcome(state: u32) -> Vec<RtdlNodeState> {
        vec![
            RtdlNodeState {
                state,
                leaf_result: Some(CapabilityCallResult {
                    error: if state == 2 { "" } else { "specific cause" }.into(),
                    ..Default::default()
                }),
                ..Default::default()
            },
            RtdlNodeState {
                state: if state == 2 { 2 } else { 3 },
                ..Default::default()
            },
        ]
    }

    #[test]
    fn all_terminal_outcomes_are_unambiguous_without_an_llm() {
        for (state, name) in [
            (2, "SUCCEEDED"),
            (3, "FAILED"),
            (4, "CANCELED"),
            (5, "TIMEOUT"),
        ] {
            let event = terminal(
                "s",
                "p",
                "探索建图",
                &outcome(state),
                state != 2,
                state == 4,
            );
            assert!(event.message.contains(name), "{}", event.message);
            assert!(event.message.contains("[plan p]"));
            assert_eq!(event.state == SessionState::Completed as u32, state == 2);
            assert_eq!(event.message.contains("specific cause"), state != 2);
            assert!(!event.message.contains("地图已保存"));
        }
    }

    #[test]
    fn missing_results_are_failure_not_success() {
        assert!(
            terminal("s", "p", "work", &[], false, false)
                .message
                .contains("FAILED")
        );
    }
}
