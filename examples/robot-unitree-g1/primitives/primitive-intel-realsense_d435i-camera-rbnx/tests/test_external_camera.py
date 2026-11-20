"""Check the laptop camera primitive without ROS or G1 hardware."""

import ast
from pathlib import Path
import unittest
from unittest.mock import Mock


class ExternalCameraTest(unittest.TestCase):
    """Verify that a remotely driven camera is not launched a second time."""

    def test_waits_for_all_local_camera_inputs_without_spawning(self):
        """Decline to declare contracts if the local depth stream is absent."""
        source = Path(__file__).resolve().parents[1] / "realsense_camera/main.py"
        tree = ast.parse(source.read_text())
        init = next(node for node in tree.body if isinstance(node, ast.FunctionDef)
                    and node.name == "init")
        init.decorator_list = []
        cap = Mock()
        cap.wait_for_topic.side_effect = [True, False]
        spawn = Mock()
        spawn_mount_tf = Mock()
        kill_mount_tf = Mock()
        error = lambda message: message
        scope = {
            "cap": cap,
            "_spawn_realsense": spawn,
            "_spawn_camera_mount_tf": spawn_mount_tf,
            "_kill_realsense": Mock(),
            "_kill_camera_mount_tf": kill_mount_tf,
            "_on_rgb": Mock(),
            "_on_depth": Mock(),
            "Err": error,
        }
        exec(compile(ast.Module(body=[init], type_ignores=[]), str(source), "exec"), scope)

        result = scope["init"]({"camera_name": "camera", "launch_realsense": False})

        self.assertIn("no Image on /camera/aligned_depth_to_color/image_raw", result)
        spawn.assert_not_called()
        spawn_mount_tf.assert_called_once_with({"camera_name": "camera", "launch_realsense": False})
        kill_mount_tf.assert_called_once_with()
        cap.declare_ros2_topic.assert_not_called()
        self.assertEqual(cap.wait_for_topic.call_count, 2)


if __name__ == "__main__":
    unittest.main()
