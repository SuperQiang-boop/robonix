"""Unit checks for cold configuration validation."""
from pathlib import Path
import sys
import unittest
from unittest.mock import patch
import tempfile
import os

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, "/home/szh/robonix/pylib/robonix-api")
from face_recognition.main import Settings  # noqa: E402


class SettingsTests(unittest.TestCase):
    def test_defaults_and_relative_binary(self):
        with tempfile.TemporaryDirectory() as root:
            binary = Path(root) / "bin" / "vsis_video_service"
            binary.parent.mkdir()
            binary.write_text("#!/bin/sh\n")
            binary.chmod(0o755)
            value = Settings.parse({"external_root": root})
        self.assertEqual(value.speak_port, 10086)
        self.assertTrue(value.binary_path.name == "vsis_video_service")

    def test_unknown_key_rejected(self):
        with self.assertRaisesRegex(ValueError, "unknown config keys"):
            Settings.parse({"external_root": "/tmp/vsis", "oops": True})

    def test_timeout_range_rejected(self):
        with self.assertRaisesRegex(ValueError, "startup_timeout"):
            Settings.parse({"external_root": "/tmp/vsis", "startup_timeout": 0})


if __name__ == "__main__":
    unittest.main()
