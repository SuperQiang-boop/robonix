"""Unit checks for cold configuration validation."""
from pathlib import Path
import io
import json
import sys
import unittest
from unittest.mock import patch, Mock
import tempfile
import os

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, "/home/szh/robonix/pylib/robonix-api")
from face_recognition.main import Settings  # noqa: E402
from face_recognition import main as bridge  # noqa: E402


class SettingsTests(unittest.TestCase):
    def test_defaults_and_relative_binary(self):
        with tempfile.TemporaryDirectory() as root:
            binary = Path(root) / "bin" / "vsis_video_service"
            binary.parent.mkdir()
            binary.write_text("#!/bin/sh\n")
            binary.chmod(0o755)
            value = Settings.parse({"external_root": root})
        self.assertTrue(value.binary_path.name == "vsis_video_service")

    def test_unknown_key_rejected(self):
        with self.assertRaisesRegex(ValueError, "unknown config keys"):
            Settings.parse({"external_root": "/tmp/vsis", "oops": True})

    def test_timeout_range_rejected(self):
        with self.assertRaisesRegex(ValueError, "startup_timeout"):
            Settings.parse({"external_root": "/tmp/vsis", "startup_timeout": 0})

    def test_upgrade_thresholds_require_four_increasing_counts(self):
        with tempfile.TemporaryDirectory() as root:
            binary = Path(root) / "bin" / "vsis_video_service"
            binary.parent.mkdir()
            binary.write_text("#!/bin/sh\n")
            binary.chmod(0o755)
            for thresholds in ([5, 15, 15, 60], [5, 15, 30], [0, 15, 30, 60], [5, True, 30, 60]):
                with self.subTest(thresholds=thresholds), self.assertRaisesRegex(ValueError, "intimacy_upgrade_encounters"):
                    Settings.parse({"external_root": root, "face": {"intimacy_upgrade_encounters": thresholds}})
            value = Settings.parse({"external_root": root, "face": {"intimacy_upgrade_encounters": [2, 4, 6, 8]}})
            self.assertEqual(value.vsis_config["face"]["intimacy_upgrade_encounters"], [2, 4, 6, 8])

    def test_managed_config_passed_to_child_and_removed_on_deactivate(self):
        with tempfile.TemporaryDirectory() as root:
            binary = Path(root) / "bin" / "vsis_video_service"
            binary.parent.mkdir()
            binary.write_text("#!/bin/sh\n")
            binary.chmod(0o755)
            settings = Settings.parse({
                "external_root": root,
                "video": {"source": "https://example.test/live.mjpg"},
                "face": {"intimacy_upgrade_encounters": [2, 4, 6, 8]},
            })
            process = Mock(stdout=io.StringIO("FACE_READY\n"))
            process.poll.return_value = 0
            with patch.object(bridge, "_settings", settings), patch.object(bridge.subprocess, "Popen", return_value=process) as spawn:
                try:
                    bridge.activate()
                    config_path = Path(spawn.call_args.kwargs["env"]["VSIS_CONFIG_PATH"])
                    self.assertTrue(config_path.is_file())
                    config = json.loads(config_path.read_text(encoding="utf-8"))
                    self.assertEqual(config["video"]["source"], "https://example.test/live.mjpg")
                    self.assertEqual(config["face"]["intimacy_upgrade_encounters"], [2, 4, 6, 8])
                    self.assertEqual(config["log"], {"mode": 2, "level": 1})
                finally:
                    bridge.deactivate()
                self.assertFalse(config_path.exists())


if __name__ == "__main__":
    unittest.main()
