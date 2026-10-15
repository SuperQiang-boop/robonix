"""Guard stops remain distinguishable from user cancellation through the real result callback."""

import ast
from collections import deque
import json
from pathlib import Path
import tempfile
import threading
from types import SimpleNamespace
import unittest
from unittest.mock import Mock

from nav2_wrapper.diagnostics import format_result_detail, summarize_blockage
from nav2_wrapper.guard_failure import (
    classify_guard_result, read_guard_failure, write_guard_failure,
)


class GuardFailureTest(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.root = Path(self.directory.name)
        self.uuid = "0123456789abcdef0123456789abcdef"

    def test_record_is_uuid_scoped_and_contains_actual_errors(self):
        """Read the exact action's record rather than a previous navigation failure."""
        write_guard_failure(self.root, self.uuid, "terminal correction exhausted",
                            (0.3, 0.4, 0.1), (0.0, 0.0, 0.0))
        record = read_guard_failure(self.root, self.uuid)
        self.assertAlmostEqual(record["xy_error_m"], 0.5)
        self.assertAlmostEqual(record["yaw_error_rad"], 0.1)
        self.assertIsNone(read_guard_failure(self.root, "f" * 32))
        self.assertFalse(list(self.root.glob("*.tmp")))

    def test_invalid_or_partial_record_is_ignored(self):
        """Reject invalid UUIDs, truncated JSON and mismatched or excessive records."""
        target = self.root / f"guard-{self.uuid}.json"
        for text in ('{', '[]', json.dumps({"goal_uuid": "f" * 32, "reason": "old"}),
                     " " * 9000):
            target.write_text(text)
            self.assertIsNone(read_guard_failure(self.root, self.uuid))
        self.assertIsNone(read_guard_failure(self.root, "../../bad"))

    def _result(self, user_cancel=False, handle_uuid=None, status=5):
        """Execute production result functions without importing or starting the ROS/Atlas bridge."""
        source = Path(__file__).parent / "nav2_wrapper" / "atlas_bridge.py"
        selected = {"_result_cb", "_goal_status_name", "_canonical_state"}
        parsed = ast.parse(source.read_text())
        module = ast.Module(body=[node for node in parsed.body
                                  if isinstance(node, ast.FunctionDef) and node.name in selected],
                            type_ignores=[])
        namespace = {
            "_GoalStatus": SimpleNamespace(STATUS_UNKNOWN=0, STATUS_ACCEPTED=1,
                STATUS_EXECUTING=2, STATUS_CANCELING=3, STATUS_SUCCEEDED=4,
                STATUS_CANCELED=5, STATUS_ABORTED=6),
            "_state_lock": threading.Lock(), "_guard_trace_dir": self.root,
            "_goal_states": {"run": {"cancel_requested": user_cancel,
                "feedback": {"distance_remaining": 0.37, "x": 0.3, "y": 0.4}}},
            "_goal_handles": {"run": SimpleNamespace(goal_id=SimpleNamespace(
                uuid=bytes.fromhex(handle_uuid or self.uuid)))},
            "_nav_diagnostics": deque(), "_local_costmap_snapshot": None,
            "_restore_speed_after_run": Mock(), "log": Mock(),
            "read_guard_failure": read_guard_failure,
            "classify_guard_result": classify_guard_result,
            "format_result_detail": format_result_detail, "summarize_blockage": summarize_blockage,
        }
        exec(compile(module, str(source), "exec"), namespace)
        namespace["_result_cb"](SimpleNamespace(result=lambda: SimpleNamespace(status=status)), "run")
        namespace["_restore_speed_after_run"].assert_called_once_with("run")
        return namespace["_goal_states"]["run"]

    def test_safety_cancel_becomes_failed_with_reason_and_final_error(self):
        """Pilot receives the real cause and does not mistake a guard stop for user intent."""
        write_guard_failure(self.root, self.uuid, "terminal correction blocked by local costmap",
                            (0.3, 0.4, 0.1), (0.0, 0.0, 0.0))
        state = self._result()
        self.assertEqual(state["state"], "FAILED")
        self.assertIn("safety_stop=terminal correction blocked", state["detail"])
        self.assertIn("xy_error_m=0.500m", state["detail"])
        self.assertIn("last_pose=(0.300,0.400)", state["detail"])

    def test_user_cancel_and_unrelated_cancel_stay_canceled(self):
        """An explicit stop wins, and a stale record cannot poison a different action."""
        write_guard_failure(self.root, self.uuid, "terminal correction exhausted")
        self.assertEqual(self._result(user_cancel=True)["state"], "CANCELED")
        self.assertEqual(self._result(handle_uuid="f" * 32)["state"], "CANCELED")

    def test_normal_success_is_unchanged(self):
        self.assertEqual(self._result(status=4)["state"], "SUCCEEDED")


if __name__ == "__main__":
    unittest.main()
