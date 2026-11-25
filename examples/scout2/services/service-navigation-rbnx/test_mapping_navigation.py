"""Exercise mapping admission and real guard logic without importing ROS or moving a robot."""
import ast
import logging
import math
import queue
import sys
import tempfile
import threading
import time
import unittest
import uuid
from collections import deque
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, patch

ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT.parent / 'service-relocalization'))
from relocalization_rbnx.coordination import read_record, write_record
from relocalization_rbnx.navigation import check, mapping_active, mapping_navigation_ready


def load_functions(path, names, namespace, class_name=None):
    """Compile actual production functions with fake ROS endpoints supplied by tests."""
    tree = ast.parse(path.read_text())
    body = tree.body
    if class_name:
        body = next(n for n in body if isinstance(n, ast.ClassDef) and n.name == class_name).body
    methods = [n for n in body if isinstance(n, ast.FunctionDef) and n.name in names]
    exec(compile(ast.Module(body=methods, type_ignores=[]), str(path), 'exec'), namespace)
    return namespace


class MappingNavigationTests(unittest.TestCase):
    def setUp(self):
        """Create a real private coordination directory and fake guard inputs."""
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        self.directory = Path(temporary.name)
        self.mapping = dict(provider_id='mapping', process_id='process-1', load_id='load-1',
                            ready=True, mode='mapping', map_id='first-map')
        self.publish_mapping()
        namespace = dict(math=math, time=time, Path=Path, Time=lambda: None,
                         TransformException=LookupError, OccupancyGrid=object, Twist=SimpleNamespace)
        names = {'_mapping_inputs_ready', '_navigation_ready', '_on_map', '_on_costmap',
                 '_write_localization_stop'}
        functions = load_functions(ROOT / 'nav2_wrapper/velocity_guard.py', names,
                                   namespace, 'VelocityGuardNode')
        guard_type = type('GuardHarness', (), {n: functions[n] for n in names})
        self.guard = guard_type()
        self.guard._lock = threading.Lock()
        self.guard._localization_dir = str(self.directory)
        self.guard._localization_check = lambda: check(self.directory)
        self.guard._mapping_identity = None
        self.guard._checked_identity = None
        self.guard._stationary = deque()
        self.guard._map_ready = True
        self.guard._odom_received_at = time.monotonic()
        self.guard._scan_received_at = time.monotonic()
        self.guard._costmap_received_at = time.monotonic()
        self.now_ns = time.time_ns()
        self.guard._odom_stamp_ns = self.now_ns
        self.guard._scan_stamp_ns = self.now_ns
        self.guard._scan_frame = 'lidar'
        self.guard.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=self.now_ns))
        transform = SimpleNamespace(
            header=SimpleNamespace(stamp=SimpleNamespace(sec=self.now_ns // 10**9, nanosec=self.now_ns % 10**9)),
            transform=SimpleNamespace(translation=SimpleNamespace(x=0., y=0., z=0.),
                rotation=SimpleNamespace(x=0., y=0., z=0., w=1.)))
        self.guard._tf_buffer = SimpleNamespace(lookup_transform=Mock(return_value=transform))
        self.guard.get_logger = lambda: logging.getLogger('mapping-test')

    def publish_mapping(self):
        """Publish a fresh fixture identity using the actual atomic record writer."""
        write_record(self.directory / 'mapping.json', self.mapping)

    def test_mapping_without_saved_map_or_relocalization_status_is_ready(self):
        """Online mapping can pass without any localization snapshot or status."""
        self.assertTrue(self.guard._navigation_ready())
        self.guard._write_localization_stop(False)
        self.assertTrue(mapping_navigation_ready(self.directory))
        self.assertFalse((self.directory / 'status.json').exists())

    def test_stale_or_missing_sensors_and_costmap_block_mapping(self):
        """Reception and source timestamps must both be fresh."""
        for field in ('_odom_received_at', '_scan_received_at', '_costmap_received_at'):
            for value in (None, time.monotonic() - 2):
                with self.subTest(field=field, value=value), patch.object(self.guard, field, value):
                    self.assertFalse(self.guard._navigation_ready())
        for field in ('_odom_stamp_ns', '_scan_stamp_ns'):
            with self.subTest(field=field), patch.object(self.guard, field, self.now_ns - 2*10**9):
                self.assertFalse(self.guard._navigation_ready())

    def test_missing_map_or_tf_blocks_mapping(self):
        """An occupancy sample and valid transform are mandatory."""
        with patch.object(self.guard, '_map_ready', False):
            self.assertFalse(self.guard._navigation_ready())
        self.guard._tf_buffer.lookup_transform.side_effect = LookupError('no transform')
        self.assertFalse(self.guard._navigation_ready())

    def test_stale_tf_blocks_mapping(self):
        """A cached transform cannot substitute for a live SLAM/odometry chain."""
        transform = self.guard._tf_buffer.lookup_transform.return_value
        transform.header.stamp.sec -= 2
        self.assertFalse(self.guard._navigation_ready())

    def test_map_reload_revokes_old_occupancy_and_ready_acknowledgement(self):
        """A new load needs a new map observation and a new guard acknowledgement."""
        self.assertTrue(self.guard._navigation_ready())
        self.guard._write_localization_stop(False)
        self.mapping['load_id'] = 'load-2'
        self.publish_mapping()
        self.assertFalse(mapping_navigation_ready(self.directory))
        self.assertFalse(self.guard._navigation_ready())
        msg = SimpleNamespace(header=SimpleNamespace(frame_id='map'),
            info=SimpleNamespace(width=2, height=2, resolution=.05), data=[0, 0, -1, 100])
        self.guard._on_map(msg)
        self.assertTrue(self.guard._navigation_ready())
        self.guard._write_localization_stop(False)
        self.assertTrue(mapping_navigation_ready(self.directory))

    def test_identity_change_during_validation_fails_closed(self):
        """A concurrent transition cannot be authorized by the previous input check."""
        def change_identity():
            self.mapping['load_id'] = 'changed-during-check'
            self.publish_mapping()
            return True
        with patch.object(self.guard, '_mapping_inputs_ready', side_effect=change_identity):
            self.assertFalse(self.guard._navigation_ready())

    def test_acknowledgement_cannot_authorize_a_different_load(self):
        """A transition between checking and publishing readiness revokes that result."""
        self.assertTrue(self.guard._navigation_ready())
        self.mapping['load_id'] = 'load-2'
        self.publish_mapping()
        self.guard._write_localization_stop(False)
        self.assertFalse(read_record(self.directory / 'navigation.json')['ready'])

    def test_transition_and_missing_heartbeat_block_mapping(self):
        """Revoke autonomous admission on invalidation or a missing owner record."""
        self.mapping['ready'] = False
        self.publish_mapping()
        self.assertFalse(self.guard._navigation_ready())
        (self.directory / 'mapping.json').unlink()
        self.assertFalse(self.guard._navigation_ready())

    def test_localization_still_requires_matching_verified_snapshot(self):
        """Verified localization must still match the exact map and load."""
        self.mapping.update(mode='localization', generation=2, database_sha256='a'*64)
        self.publish_mapping()
        self.assertFalse(self.guard._navigation_ready())

        status = dict(provider_id='relocalization', active=True, busy=False,
            localization_valid=True, map_id='first-map', generation=2,
            database_sha256='a'*64, mapping_process_id='process-1', load_id='load-1')
        write_record(self.directory / 'status.json', status)
        self.assertTrue(self.guard._navigation_ready())
        status['load_id'] = 'old-load'
        write_record(self.directory / 'status.json', status)
        self.assertFalse(self.guard._navigation_ready())

    def test_expired_mapping_and_guard_acknowledgements_block_admission(self):
        """A frozen heartbeat or guard cannot leave cached motion permission."""
        self.assertTrue(self.guard._navigation_ready())
        self.guard._write_localization_stop(False)
        current = time.monotonic_ns()
        with patch('relocalization_rbnx.coordination.time.monotonic_ns', return_value=current+400_000_000):
            self.assertFalse(mapping_navigation_ready(self.directory))
        with patch('relocalization_rbnx.coordination.time.monotonic_ns', return_value=current+800_000_000):
            self.assertFalse(self.guard._navigation_ready())

    def wrapper(self):
        """Use actual queue admission/dispatch with fake action and speed endpoints."""
        namespace = dict(math=math, time=time, uuid=uuid, log=logging.getLogger('mapping-test'),
            _ros_node=object(), _map_ready=True, _nav_action_ready=True,
            _localization_check=lambda: check(self.directory),
            _mapping_active_check=lambda: mapping_active(self.directory),
            _mapping_navigation_check=lambda: mapping_navigation_ready(self.directory),
            _map_load_identity=lambda: (self.mapping['process_id'], self.mapping['load_id'], self.mapping['mode']),
            _costmap_lock=threading.Lock(), _costmap_width=2, _costmap_height=2,
            _costmap_resolution=.05, _costmap_data=b'\0'*4, _costmap_updated_at=time.monotonic(),
            _state_lock=threading.Lock(), _goal_states={}, _last_run_id='', _nav_queue=queue.Queue(),
            _adjust_goal_if_needed=lambda x,y:(x,y), _adjust_goal_for_objects=lambda x,y:(x,y),
            _restore_speed_after_run=Mock(), _make_pose=Mock(return_value=object()),
            _NavigateToPose=SimpleNamespace(Goal=SimpleNamespace), _nav_action_client=Mock(),
            _nav_diagnostics=deque(), _feedback_cb=Mock(), _goal_response_cb=Mock())
        return load_functions(ROOT / 'nav2_wrapper/atlas_bridge.py',
            {'_mapping_costmap_ready', '_navigate_impl', '_dispatch_goal', '_quat_to_yaw'}, namespace)

    @staticmethod
    def goal():
        """Construct the minimal contract goal consumed by navigation admission."""
        return SimpleNamespace(header=SimpleNamespace(frame_id='map'),
            pose=SimpleNamespace(position=SimpleNamespace(x=1.,y=2.),
                orientation=SimpleNamespace(z=0.,w=1.)))

    def test_queue_admission_requires_guard_costmap_map_and_action_readiness(self):
        """Admission and dispatch use real guard results with fake ROS actions."""
        wrapper = self.wrapper()
        self.assertFalse(wrapper['_navigate_impl'](self.goal())['accepted'])
        self.assertTrue(self.guard._navigation_ready())
        self.guard._write_localization_stop(False)
        for field, value in (('_map_ready', False), ('_nav_action_ready', False),
                ('_costmap_updated_at', time.monotonic()-2), ('_costmap_data', b'')):
            with self.subTest(field=field), patch.dict(wrapper, {field: value}):
                self.assertFalse(wrapper['_navigate_impl'](self.goal())['accepted'])
        accepted = wrapper['_navigate_impl'](self.goal())
        self.assertTrue(accepted['accepted'])
        gid, payload = wrapper['_nav_queue'].get_nowait()
        wrapper['_dispatch_goal'](object(), gid, payload)
        wrapper['_nav_action_client'].send_goal_async.assert_called_once()

    def test_queued_goal_cannot_cross_map_reload(self):
        """Do not reinterpret queued coordinates in the next map frame."""
        self.assertTrue(self.guard._navigation_ready())
        self.guard._write_localization_stop(False)
        wrapper = self.wrapper()
        self.assertTrue(wrapper['_navigate_impl'](self.goal())['accepted'])
        gid, payload = wrapper['_nav_queue'].get_nowait()
        self.mapping['load_id'] = 'load-2'
        self.publish_mapping()
        wrapper['_dispatch_goal'](object(), gid, payload)
        self.assertEqual(wrapper['_goal_states'][gid]['state'], 'FAILED')
        wrapper['_nav_action_client'].send_goal_async.assert_not_called()

    def test_queued_goal_rechecks_guard_and_costmap(self):
        """Lost readiness after admission prevents an action from being sent."""
        self.assertTrue(self.guard._navigation_ready())
        self.guard._write_localization_stop(False)
        wrapper = self.wrapper()
        self.assertTrue(wrapper['_navigate_impl'](self.goal())['accepted'])
        gid, payload = wrapper['_nav_queue'].get_nowait()
        wrapper['_costmap_updated_at'] -= 2
        wrapper['_dispatch_goal'](object(), gid, payload)
        self.assertEqual(wrapper['_goal_states'][gid]['state'], 'FAILED')
        wrapper['_nav_action_client'].send_goal_async.assert_not_called()

    def test_guard_receives_atlas_bindings_and_drops_previous_scan_binding(self):
        """Use the selected providers' topics even when a previous scan differs."""
        environment = {'ROBONIX_NAV_GUARD_SCAN_TOPIC': '/old/scan'}
        captured = []
        namespace = dict(os=SimpleNamespace(environ=environment),
            subprocess=Mock(), threading=Mock(), log=Mock(), _pump_output=Mock(),
            _materialize_params=lambda cfg, bindings: ('params.yaml', []),
            _materialize_guarded_launch=lambda: 'launch.py',
            _spawn_velocity_guard=lambda cfg: captured.append(dict(environment)))
        wrapper = load_functions(ROOT / 'nav2_wrapper/atlas_bridge.py',
            {'_spawn_nav2', '_binding_value'}, namespace)
        wrapper['_spawn_nav2']({'use_sim_time': True},
            ['map:=/fixture/map', 'odom:=/fixture/odom', 'scan:=/fixture/scan'])
        self.assertEqual(captured[-1]['ROBONIX_NAV_GUARD_MAP_TOPIC'], '/fixture/map')
        self.assertEqual(captured[-1]['ROBONIX_NAV_GUARD_ODOM_TOPIC'], '/fixture/odom')
        self.assertEqual(captured[-1]['ROBONIX_NAV_GUARD_SCAN_TOPIC'], '/fixture/scan')
        self.assertEqual(captured[-1]['ROBONIX_NAV_GUARD_USE_SIM_TIME'], 'true')
        wrapper['_spawn_nav2']({}, ['map:=/fixture/map', 'odom:=/fixture/odom'])
        self.assertNotIn('ROBONIX_NAV_GUARD_SCAN_TOPIC', captured[-1])


if __name__ == '__main__':
    unittest.main()
