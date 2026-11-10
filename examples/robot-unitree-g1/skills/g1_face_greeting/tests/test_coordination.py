"""Offline regressions for the opt-in greeting/Planner handoff."""
import json
import os
import sys
import tempfile
import time
import unittest
from pathlib import Path
from unittest.mock import patch

PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE))
sys.path.insert(0, str(PACKAGE / "rbnx-build/codegen/robonix_mcp_types"))
sys.path.insert(0, str(PACKAGE / "rbnx-build/codegen/proto_gen"))
sys.path.insert(0, str(PACKAGE.parents[3] / "pylib/robonix-api"))

from g1_face_greeting import main, coordination


class CoordinationTests(unittest.TestCase):
    def setUp(self):
        """Use a real atomic snapshot with fake navigation, audio and task ownership."""
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        self.path = Path(directory.name) / "greeting.json"
        environment = patch.dict(os.environ, {coordination.STATE_ENV: str(self.path)})
        environment.start()
        self.addCleanup(environment.stop)
        main._settings = main.Settings.parse({"navigation_enabled": True})
        main._last_greet.clear()
        main._seen_events.clear()
        main._stop.clear()
        self.event = {"person_id": 12, "event_id": "encounter", "intimacy_level": 3}
        for target, kwargs in [
            ("owner", {"return_value": {"session_id": "original-session", "plan_id": "A"}}),
        ]:
            mocked = patch.object(coordination, target, **kwargs)
            mocked.start()
            self.addCleanup(mocked.stop)
        navigation = patch.object(main, "_navigation_call", side_effect=lambda tool, _: (
            {"accepted": True, "run_id": "run-A"} if tool == "pause"
            else {"state": "PAUSED", "detail": "navigation paused; stationary=true"}))
        self.nav = navigation.start()
        self.addCleanup(navigation.stop)
        speech = patch.object(main, "_greet_stationary", side_effect=lambda _: {
            "accepted": True, "speech_ok": True, "action_ok": True, "detail": "speech accepted"})
        self.speech = speech.start()
        self.addCleanup(speech.stop)
        for target, kwargs in [("_wait_for_greeting", {}),
                               ("_release_arm", {"return_value": {"ok": True, "detail": "released"}})]:
            mocked = patch.object(main, target, **kwargs)
            mocked.start()
            self.addCleanup(mocked.stop)

    def read(self):
        return json.loads(self.path.read_text())

    def test_greeting_hands_task_back_without_resuming_original(self):
        """A voice command is admitted under the original session while motion waits."""
        def listen(runtime, facts):
            self.assertEqual(facts["session_id"], "original-session")
            self.assertEqual(facts["run_id"], "run-A")
            self.assertTrue(self.read()["listening"])
            self.assertFalse(self.read()["movement_allowed"])
            return True

        with patch.object(coordination, "listen", side_effect=listen):
            result = main._greet(self.event)
        self.assertTrue(result["accepted"])
        self.assertTrue(self.speech.call_args.args[0]["ask_for_task"])
        self.assertEqual([call.args[0] for call in self.nav.call_args_list], ["pause", "status"])
        self.assertTrue(self.read()["movement_allowed"])
        self.assertEqual(self.read()["state"], "SUCCEEDED")

    def test_no_voice_request_still_leaves_resume_to_planner(self):
        with patch.object(coordination, "listen", return_value=False):
            main._greet(self.event)
        self.assertIn("no voice request", self.read()["detail"])
        self.assertNotIn("resume", [call.args[0] for call in self.nav.call_args_list])

    def test_failed_arm_release_never_listens_or_permits_new_goal(self):
        """Verify test failed arm release never listens or permits new goal."""
        with patch.object(main, "_release_arm", return_value={"ok": False, "detail": "timeout"}), \
                patch.object(coordination, "listen") as listen:
            main._greet(self.event)
        listen.assert_not_called()
        self.assertFalse(self.read()["movement_allowed"])
        self.assertEqual(self.read()["state"], "FAILED")

    def test_unknown_voice_admission_stays_stopped(self):
        with patch.object(coordination, "listen", side_effect=RuntimeError("access denied")):
            main._greet(self.event)
        self.assertFalse(self.read()["movement_allowed"])
        self.assertIn("access denied", self.read()["detail"])

    def test_speech_only_keeps_handoff_without_arm_requests(self):
        """Verify test speech only keeps handoff without arm requests."""
        with patch.object(main, "ARM_GREETING_ENABLED", False), \
                patch.object(main, "_release_arm") as release, \
                patch.object(coordination, "listen", return_value=False):
            main._greet(self.event)
        release.assert_not_called()
        self.assertTrue(self.read()["movement_allowed"])

    def test_stop_during_greeting_does_not_admit_late_voice(self):
        """Verify test stop during greeting does not admit late voice."""
        def stop_after_wait(*_):
            Path(str(self.path) + ".stop").write_text(json.dumps({
                "session_id": "original-session", "event_id": "encounter"}))

        with patch.object(main, "_wait_for_greeting", side_effect=stop_after_wait), \
                patch.object(coordination, "listen") as listen:
            main._greet(self.event)
        listen.assert_not_called()
        self.assertTrue(self.read()["movement_allowed"])
        self.assertIn("canceled by Stop", self.read()["detail"])
        self.assertNotIn("resume", [call.args[0] for call in self.nav.call_args_list])

    def test_restart_never_erases_unconfirmed_greeting(self):
        coordination.publish({"movement_allowed": False, "detail": "release unknown"})
        with self.assertRaisesRegex(RuntimeError, "previous greeting"):
            coordination.initialize()
        self.assertFalse(self.read()["movement_allowed"])

    def test_cooldown_does_not_change_snapshot_or_interrupt_navigation(self):
        main._last_greet[12] = time.monotonic()
        result = main._greet(self.event)
        self.assertFalse(result["accepted"])
        self.assertFalse(self.path.exists())
        self.nav.assert_not_called()
