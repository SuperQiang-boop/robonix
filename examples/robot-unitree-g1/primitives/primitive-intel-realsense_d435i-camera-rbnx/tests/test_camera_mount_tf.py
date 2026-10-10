"""Verify camera mount TF process arguments without ROS installed."""

import ast
from pathlib import Path
import unittest
from unittest.mock import Mock


class CameraMountTfTest(unittest.TestCase):
    def setUp(self):
        source = Path(__file__).resolve().parents[1] / "realsense_camera/main.py"
        tree = ast.parse(source.read_text())
        functions = [
            node for node in tree.body
            if isinstance(node, ast.FunctionDef)
            and node.name in {"_spawn_camera_mount_tf", "_kill_camera_mount_tf"}
        ]
        self.process = Mock()
        self.popen = Mock(return_value=self.process)
        self.scope = {"subprocess": Mock(Popen=self.popen), "log": Mock()}
        exec(compile(ast.Module(body=functions, type_ignores=[]), str(source), "exec"),
             self.scope)

    def test_starts_configured_static_transform(self):
        self.scope["_spawn_camera_mount_tf"]({})

        args = self.popen.call_args.args[0]
        self.assertEqual(args[:4], ["ros2", "run", "tf2_ros", "static_transform_publisher"])
        self.assertEqual(args[args.index("--frame-id") + 1], "d435_link")
        self.assertEqual(args[args.index("--child-frame-id") + 1], "camera_link")
        self.assertEqual(args[args.index("--x") + 1], "0.0")
        self.popen.assert_called_once_with(args, start_new_session=True)

    def test_can_disable_transform_publisher(self):
        self.scope["_spawn_camera_mount_tf"]({"publish_camera_mount_tf": False})

        self.popen.assert_not_called()


if __name__ == "__main__":
    unittest.main()
