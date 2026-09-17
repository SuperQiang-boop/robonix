"""Exercise the actual TF worker without requiring ROS or camera hardware."""
import ast
import logging
from pathlib import Path
import threading
import types
import unittest
from unittest.mock import Mock, patch


class ExtrinsicsRetryTest(unittest.TestCase):
    """Load only the worker so ROS and generated MCP imports can be stubbed."""

    def setUp(self):
        """Compile the production worker and supply mocked ROS dependencies."""
        source = Path(__file__).resolve().parents[1] / "realsense_camera/main.py"
        tree = ast.parse(source.read_text())
        worker = next(n for n in tree.body if isinstance(n, ast.FunctionDef)
                      and n.name == "_publish_extrinsics_when_ready")
        self.logger = Mock(spec=logging.Logger)
        self.clock = Mock()
        self.scope = dict(threading=threading, time=self.clock, log=self.logger)
        exec(compile(ast.Module(body=[worker], type_ignores=[]), str(source), "exec"),
             self.scope)
        self.buffer = Mock()
        self.listener = Mock()
        self.modules = {
            "rclpy.time": types.SimpleNamespace(Time=Mock()),
            "tf2_ros": types.SimpleNamespace(
                Buffer=Mock(return_value=self.buffer),
                TransformListener=Mock(return_value=self.listener)),
            "robonix_api.ros": types.SimpleNamespace(RosBackend=Mock()),
        }
        self.publisher = Mock()
        self.stop = threading.Event()

    def run_worker(self):
        """Run the worker with the same explicit lifetime arguments as init."""
        with patch.dict("sys.modules", self.modules):
            self.scope["_publish_extrinsics_when_ready"](
                "base_link", "camera_color_optical_frame", "/camera/extrinsics",
                self.publisher, self.stop)

    def test_recovers_after_sixty_seconds_and_throttles_errors(self):
        """A TF arriving after the former deadline is still published once."""
        transform = Mock()
        transform.transform.translation = types.SimpleNamespace(x=1., y=2., z=3.)
        self.buffer.lookup_transform.side_effect = [
            RuntimeError("camera frame missing"), RuntimeError("camera frame missing"),
            RuntimeError("TF trees disconnected"), transform]
        self.clock.monotonic.side_effect = [0., 5., 70.]
        with patch.object(self.stop, "wait", return_value=False):
            self.run_worker()
        self.publisher.publish.assert_called_once_with(transform)
        self.assertEqual(transform.header.frame_id, "base_link")
        self.assertEqual(transform.child_frame_id, "camera_color_optical_frame")
        self.assertEqual(self.logger.warning.call_count, 2)
        self.assertIn("TF trees disconnected", str(self.logger.warning.call_args))
        self.listener.unregister.assert_called_once()

    def test_shutdown_stops_retry_without_publishing(self):
        """Shutdown during a failed lookup exits and releases TF subscriptions."""
        self.buffer.lookup_transform.side_effect = RuntimeError("missing TF")
        self.clock.monotonic.return_value = 0.
        with patch.object(self.stop, "wait", side_effect=lambda _: self.stop.set()):
            self.run_worker()
        self.buffer.lookup_transform.assert_called_once()
        self.publisher.publish.assert_not_called()
        self.listener.unregister.assert_called_once()


if __name__ == "__main__":
    unittest.main()
