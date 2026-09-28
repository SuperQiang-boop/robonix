import sys
import threading
import time
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE))
sys.path.insert(0, str(PACKAGE / "rbnx-build/codegen/robonix_mcp_types"))
sys.path.insert(0, str(PACKAGE.parents[3] / "pylib/robonix-api"))

from g1_face_greeting import main  # noqa: E402


class GreetingTests(unittest.TestCase):
    def setUp(self):
        main._settings = main.Settings.parse({"cooldown_sec": 30})
        main._speech_connection = SimpleNamespace(endpoint="speech")
        main._arm_connection = SimpleNamespace(endpoint="arm")
        main._last_greet.clear()
        main._seen_events.clear()

    def test_config_keeps_navigation_and_turning_disabled(self):
        settings = main.Settings.parse({})
        self.assertFalse(settings.navigation_enabled)
        self.assertFalse(settings.turn_to_face_enabled)
        with self.assertRaisesRegex(ValueError, "must remain false"):
            main.Settings.parse({"navigation_enabled": True})
        with self.assertRaisesRegex(ValueError, "must remain false"):
            main.Settings.parse({"turn_to_face_enabled": True})

    def test_intimacy_levels_select_allowlisted_actions(self):
        expected = {
            1: "wave_under_head",
            2: "wave_under_head",
            3: "wave_above_head",
            4: "make_heart_with_right_hand",
            5: "make_heart_with_both_hands",
        }
        for level, action in expected.items():
            with self.subTest(level=level):
                main._last_greet.clear()
                main._seen_events.clear()
                def call(connection, tool, payload, timeout):
                    if tool == "speak":
                        return {"ok": True}
                    return {"accepted": True}

                with patch.object(main, "_call_sync", side_effect=call) as call_mock:
                    result = main._greet({"event_id": str(level), "person_id": level, "intimacy_level": level})
                self.assertTrue(result["accepted"])
                self.assertEqual(result["action"], action)
                action_calls = [item for item in call_mock.call_args_list if item.args[1] == "run"]
                self.assertEqual(action_calls[0].args[2], {"action": action})

    def test_invalid_levels_are_rejected_without_side_effects(self):
        for level in (0, 6, 1.5, "2", True, None):
            with self.subTest(level=level):
                with patch.object(main, "_call_sync") as call:
                    result = main._greet({"person_id": 1, "intimacy_level": level})
                self.assertFalse(result["accepted"])
                call.assert_not_called()

    def test_duplicate_event_and_person_cooldown_are_suppressed(self):
        def call(connection, tool, payload, timeout):
            if tool == "speak":
                return {"ok": True}
            return {"accepted": True}

        with patch.object(main, "_call_sync", side_effect=call) as call_mock:
            accepted = main._greet({"event_id": "event-1", "person_id": 7, "intimacy_level": 1})
            duplicate = main._greet({"event_id": "event-1", "person_id": 7, "intimacy_level": 1})
            cooldown = main._greet({"event_id": "event-2", "person_id": 7, "intimacy_level": 1})
        self.assertTrue(accepted["accepted"])
        self.assertEqual(duplicate["detail"], "duplicate event")
        self.assertEqual(cooldown["detail"], "greeting cooldown")
        self.assertEqual(call_mock.call_count, 2)

    def test_speech_and_arm_failures_are_reported_without_retry(self):
        def call(connection, tool, payload, timeout):
            if tool == "speak":
                raise RuntimeError("speech offline")
            return {"accepted": False, "detail": "busy"}

        with patch.object(main, "_call_sync", side_effect=call) as call_mock:
            result = main._greet({"event_id": "failure", "person_id": 9, "intimacy_level": 2})
        self.assertTrue(result["accepted"])
        self.assertFalse(result["speech_ok"])
        self.assertFalse(result["action_ok"])
        self.assertIn("speech_error=speech offline", result["detail"])
        self.assertIn("action=busy", result["detail"])
        self.assertEqual(call_mock.call_count, 2)

    def test_speech_and_arm_start_without_waiting_for_each_other(self):
        started = {"speak": threading.Event(), "run": threading.Event()}
        release = threading.Event()

        def call(connection, tool, payload, timeout):
            started[tool].set()
            self.assertTrue(release.wait(timeout=1))
            return {"ok": True} if tool == "speak" else {"accepted": True}

        with patch.object(main, "_call_sync", side_effect=call):
            result_holder = {}
            thread = threading.Thread(
                target=lambda: result_holder.setdefault(
                    "result",
                    main._greet({"event_id": "parallel", "person_id": 10, "intimacy_level": 1}),
                )
            )
            thread.start()
            self.assertTrue(started["speak"].wait(timeout=1))
            self.assertTrue(started["run"].wait(timeout=1))
            release.set()
            thread.join(timeout=2)

        self.assertFalse(thread.is_alive())
        self.assertTrue(result_holder["result"]["speech_ok"])
        self.assertTrue(result_holder["result"]["action_ok"])

    def test_mcp_result_reads_structured_or_text_json(self):
        structured = SimpleNamespace(structuredContent={"available": True}, content=[])
        text = SimpleNamespace(structuredContent=None, content=[SimpleNamespace(text='{"ok":true}')])
        self.assertEqual(main._result_value(structured), {"available": True})
        self.assertEqual(main._result_value(text), {"ok": True})

    def test_event_poll_uses_contract_leaf_as_mcp_tool_name(self):
        main._face_connection = SimpleNamespace(endpoint="face")
        main._stop.clear()

        def one_poll(connection, tool, payload, timeout):
            main._stop.set()
            return {"available": False}

        try:
            with patch.object(main, "_call_sync", side_effect=one_poll) as call:
                main._worker_loop()
            self.assertEqual(call.call_args.args[1], "next")
            self.assertEqual(call.call_args.args[2], {"wait_ms": 1000})
        finally:
            main._stop.clear()


if __name__ == "__main__":
    unittest.main()
