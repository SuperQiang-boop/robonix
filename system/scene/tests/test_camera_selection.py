"""Camera discovery checks without starting Atlas, ROS, or perception models."""
import ast
import logging
import unittest
from enum import IntEnum
from pathlib import Path
from types import SimpleNamespace
from typing import Optional
from unittest.mock import Mock


class CameraSelectionTests(unittest.TestCase):
    """Exercise the production discovery functions with two registered cameras."""

    def setUp(self):
        """Load discovery functions alone to keep these checks hardware independent."""
        source = (Path(__file__).resolve().parents[1] / 'scene_service/service.py').read_text()
        names = {'_build_topic_specs', '_resolve_auto', '_resolve_explicit',
                 '_resolve_one_contract', '_resolve_pb_transport'}
        tree = ast.parse(source)
        tree.body = [node for node in tree.body if isinstance(node, ast.FunctionDef)
                     and node.name in names]
        transport = IntEnum('Transport', {'ROS2': 1, 'GRPC': 2})
        cameras = [SimpleNamespace(provider_id=name) for name in
                   ['face_camera', 'realsense_camera']]
        atlas = Mock()
        atlas.find_capability.side_effect = lambda **kw: [
            cap for cap in cameras
            if not kw.get('provider_id') or cap.provider_id == kw['provider_id']]
        scene = Mock()
        scene.connect_capability.side_effect = lambda cap, *_: SimpleNamespace(
            endpoint=('/camera/color/image_raw' if cap.provider_id == 'realsense_camera'
                      else '/face/camera/color/image_raw'), params=None)
        self.env = dict(Transport=transport, Optional=Optional, TopicSpec=lambda **kw: SimpleNamespace(**kw),
                        ATLAS=atlas, scene=scene, Ros2Params=type('Ros2Params', (), {}),
                        log=logging.getLogger(__name__), _LAST_RESOLVED={},
                        _TRANSPORTS={'ros2': transport.ROS2, 'grpc': transport.GRPC},
                        _DEFAULT_DISABLED_KINDS=set(),
                        _SCENE_CONTRACTS=[('rgb', 'robonix/primitive/camera/rgb', 'Image')])
        exec(compile(tree, '<scene discovery>', 'exec'), self.env)

    def test_auto_discovery_selects_realsense_with_face_camera_first(self):
        specs = self.env['_build_topic_specs']([], None, 'ros2', {'rgb': 'realsense_camera'})
        self.assertEqual(specs[0].topic, '/camera/color/image_raw')

    def test_explicit_observation_uses_same_camera_filter(self):
        specs = self.env['_build_topic_specs'](
            [{'kind': 'rgb', 'contract': 'robonix/primitive/camera/rgb'}],
            None, 'ros2', {'rgb': 'realsense_camera'})
        self.assertEqual(specs[0].topic, '/camera/color/image_raw')

    def test_missing_selected_camera_never_uses_face_camera(self):
        specs = self.env['_build_topic_specs']([], None, 'ros2', {'rgb': 'missing_camera'})
        self.assertEqual(specs, [])
        self.env['scene'].connect_capability.assert_not_called()

    def test_unconfigured_discovery_keeps_existing_behavior(self):
        specs = self.env['_build_topic_specs']([], None, 'ros2')
        self.assertEqual(specs[0].topic, '/face/camera/color/image_raw')


if __name__ == '__main__':
    unittest.main()
