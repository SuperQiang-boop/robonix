"""Data conversion and invalid configuration regressions without ROS hardware."""
import unittest
from types import SimpleNamespace
from camera_rtsp.runtime import Settings, rgb_frame, terminate
import subprocess
import sys


class RuntimeTests(unittest.TestCase):
    def test_color_and_padding(self):
        """Distinguish all three color channels and discard row padding."""
        msg = SimpleNamespace(width=1, height=2, step=4, encoding="bgr8",
                              data=bytes([1, 2, 3, 99, 4, 5, 6, 99]))
        self.assertEqual(rgb_frame(msg), bytes([3, 2, 1, 6, 5, 4]))
        msg.encoding = "rgba8"
        self.assertEqual(rgb_frame(msg), bytes([1, 2, 3, 4, 5, 6]))
        msg.data = b"short"
        with self.assertRaises(ValueError):
            rgb_frame(msg)

    def test_config(self):
        """Reject ambiguous configuration and invalid timeout/rate values."""
        base = dict(camera_provider_id="camera", rtsp_url="rtsp://127.0.0.1:8554/camera")
        self.assertTrue(Settings.parse(base).manage_server)
        for key, value in [("fps", True), ("fps", 0), ("frame_timeout", float("nan")),
                           ("manage_server", "false"), ("topic", "/hardcoded"),
                           ("rtsp_url", "rtsp://user:pass@host/camera")]:
            with self.subTest(key=key, value=value), self.assertRaises((ValueError, TypeError)):
                Settings.parse({**base, key: value})

    def test_child_reaped(self):
        """Stopping an owned child is bounded and repeatable."""
        child = subprocess.Popen([sys.executable, "-c", "import time; time.sleep(60)"])
        terminate(child)
        terminate(child)
        self.assertIsNotNone(child.returncode)


if __name__ == "__main__":
    unittest.main()
