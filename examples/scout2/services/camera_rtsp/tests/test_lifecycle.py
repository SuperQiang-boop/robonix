"""Exercise activation rollback and repeated lifecycle without a live robot."""
import unittest
from unittest.mock import Mock, patch
from camera_rtsp import main
from camera_rtsp.runtime import Settings


class LifecycleTests(unittest.TestCase):
    def tearDown(self):
        main._pipeline = None
        main._settings = None
        main._active = False

    def test_repeat_and_rollback(self):
        """Each activation reconnects; failed readiness closes the acquired pipeline."""
        main._settings = Settings("camera", "rtsp://127.0.0.1:8554/camera")
        with patch.object(main, "discover"), patch.object(main.service, "connect_capability"), \
                patch.object(main, "Pipeline") as factory:
            pipeline = factory.return_value
            for _ in range(2):
                main.activate()
                self.assertTrue(main._active)
                main.deactivate()
                self.assertFalse(main._active)
            self.assertEqual(pipeline.close.call_count, 2)
            pipeline.start.side_effect = RuntimeError("no decoded frame")
            main.activate()
            self.assertIsNone(main._pipeline)
            self.assertFalse(main._active)
            self.assertEqual(pipeline.close.call_count, 3)

    def test_failure_never_reports_url(self):
        """An asynchronous failure must not return an apparently usable URL."""
        main._settings = Settings("camera", "rtsp://127.0.0.1:8554/camera")
        main._active = True
        main._pipeline = Mock(ready=True, error="camera disconnected")
        main._pipeline.healthy.return_value = False
        response = main.get_stream(None, Mock())
        self.assertFalse(response.available)
        self.assertEqual(response.url, "")
        self.assertEqual(response.error, "camera disconnected")
