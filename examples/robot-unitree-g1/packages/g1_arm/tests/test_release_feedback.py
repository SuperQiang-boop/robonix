"""Compile the real action CLI with fake transport; never connect to hardware."""

import os
import platform
import subprocess
import tempfile
import unittest
from pathlib import Path


class ReleaseFeedbackTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        """Build production parsing/control flow against SDK JSON and fake transport."""
        package = Path(__file__).resolve().parents[1]
        sdk = Path(os.environ.get("UNITREE_SDK2_DIR", package.parents[1] / "third_party/unitree_sdk2"))
        temporary = tempfile.TemporaryDirectory(prefix="arm-feedback-test-")
        cls.addClassCleanup(temporary.cleanup)
        cls.binary = Path(temporary.name) / "g1_arm_action_cli"
        subprocess.run([
            "c++", "-std=c++17", "-Wall", "-Wextra", "-Wpedantic", "-pthread",
            "-I", str(package / "tests/fakes"),
            "-isystem", str(sdk / "include"),
            "-isystem", str(sdk / "thirdparty/include"),
            "-isystem", str(sdk / "thirdparty/include/ddscxx"),
            str(package / "src/g1_arm_action_cli.cpp"),
            str(sdk / "lib" / platform.machine() / "libunitree_sdk2.a"),
            "-o", str(cls.binary),
        ], check=True, capture_output=True, text=True)

    def run_cli(self, scenario, action="release_arm"):
        """Run one bounded scenario and preserve stdout/stderr for assertions."""
        return subprocess.run([
            str(self.binary), "--interface", "fake", "--action", action,
            "--motion-ack", "G1_ARM_ACTIONS_APPROVED",
        ], env={**os.environ, "ARM_TEST_SCENARIO": scenario},
            capture_output=True, text=True, timeout=5)

    def test_feedback_before_or_after_reply_confirms_release(self):
        """A successful RPC accepts new release feedback on either side of its reply."""
        for scenario in ("before_reply", "after_reply"):
            with self.subTest(scenario=scenario):
                result = self.run_cli(scenario)
                self.assertEqual(result.returncode, 0, result.stderr)
                self.assertIn("release_arm confirmed by action state: id=99 holding=false", result.stdout)

    def test_rpc_failure_cannot_be_overridden_by_release_feedback(self):
        """Either controller release format still requires a successful release RPC."""
        for scenario in ("rpc_failure", "idle_rpc_failure"):
            result = self.run_cli(scenario)
            self.assertEqual(result.returncode, 3)
            self.assertIn("returned 7401", result.stderr)
            self.assertNotIn("confirmed by action state", result.stdout)

    def test_firmware_idle_feedback_can_confirm_release(self):
        """Repeated fresh id=0/name-empty feedback supports this firmware's idle output."""
        for scenario in ("idle_before_reply", "idle_after_reply"):
            result = self.run_cli(scenario)
            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertIn('release_arm confirmed by action state: id=0 holding=false name=""; idle confirmed for >=0.2s', result.stdout)

    def test_idle_feedback_must_be_fresh_sustained_and_unambiguous(self):
        """Old, brief, stale, holding or named activity must never authorize motion."""
        for scenario in ("old_idle", "idle_single", "stale_idle", "idle_superseded",
                         "idle_holding", "idle_named", "idle_missing_name"):
            with self.subTest(scenario=scenario):
                result = self.run_cli(scenario)
                self.assertEqual(result.returncode, 4, result.stderr)
                self.assertIn('not confirmed within 2s', result.stderr)
                self.assertNotIn('confirmed by action state', result.stdout)

    def test_missing_stale_or_invalid_feedback_cannot_confirm_release(self):
        """Cached samples, wrong/holding state and malformed messages fail with evidence."""
        for scenario, evidence in (("old_only", "received_after_request=0"),
                                   ("none", "received_after_request=0"),
                                   ("holding", "last_valid_state=id=99 holding=true"),
                                   ("wrong_id", "last_valid_state=id=25 holding=false"),
                                   ("superseded", "last_valid_state=id=99 holding=true"),
                                   ("malformed", "invalid=1")):
            with self.subTest(scenario=scenario):
                result = self.run_cli(scenario)
                self.assertEqual(result.returncode, 4, result.stderr)
                self.assertIn(evidence, result.stderr)
                self.assertNotIn("confirmed by action state", result.stdout)

    def test_other_presets_keep_acceptance_behavior(self):
        result = self.run_cli("none", "wave_under_head")
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("SDK2 action accepted: wave_under_head", result.stdout)
        self.assertNotIn("release_arm confirmed", result.stdout)
