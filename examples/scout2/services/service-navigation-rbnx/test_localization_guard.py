"""Run the actual velocity guard methods without starting ROS or a chassis."""
import ast
import copy
import math
import threading
import unittest
from pathlib import Path
from types import ModuleType, SimpleNamespace
from unittest.mock import Mock, patch


class LocalizationGuardTests(unittest.TestCase):
    def setUp(self):
        """Extract guard methods with fake publishers; never emit real velocity."""
        path = Path(__file__).parent / 'nav2_wrapper/velocity_guard.py'
        tree = ast.parse(path.read_text())
        definition = next(node for node in tree.body if isinstance(node, ast.ClassDef)
                          and node.name == 'VelocityGuardNode')
        methods = [node for node in definition.body if isinstance(node, ast.FunctionDef)
                   and node.name in ('_publish_latched_zero', '_on_cmd')]
        self.namespace = dict(Twist=SimpleNamespace, copy=copy, math=math)
        exec(compile(ast.Module(body=methods, type_ignores=[]), str(path), 'exec'), self.namespace)
        self.timer = self.namespace['_publish_latched_zero']
        self.command = self.namespace['_on_cmd']
        self.namespace['bounded_linear_velocity'] = self._bounded_velocity
        self.mapping = True
        self.inputs_ready = True
        module = ModuleType('relocalization_rbnx.navigation')
        module.mapping_active = lambda directory: self.mapping
        patcher = patch.dict('sys.modules', {'relocalization_rbnx.navigation': module})
        patcher.start()
        self.addCleanup(patcher.stop)
        self.node = SimpleNamespace(_lock=threading.Lock(), _localization_dir='/fixture',
            _localization_check=lambda: (True, ''), _pub=Mock(),
            _cancel_for_localization=Mock(), _write_localization_stop=Mock(),
            _mapping_inputs_ready=lambda: self.inputs_ready,
            _localization_angular=None, _effective_linear_speed_mps=0.2,
            _last_cmd=(0.0, 0.0), _active_uuid='',
            _robot_pose=None, _goal_pose=None, _cancel_sent=False,
            guard=SimpleNamespace(latched_reason='', terminal_rotation=False,
                spin_rotation=0.0, evaluate=lambda *args: None),
            get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=1_000_000_000)),
            _event=Mock(), _trip=Mock())
        self.node._navigation_ready = lambda: (
            self.node._localization_check()[0] and
            (not self.mapping or self.inputs_ready))

    @staticmethod
    def _command():
        """Create a Nav2 command with the fields consumed by the guard."""
        return SimpleNamespace(
            linear=SimpleNamespace(x=0.1, y=0.0, z=0.0),
            angular=SimpleNamespace(x=0.0, y=0.0, z=0.0),
        )

    @staticmethod
    def _bounded_velocity(x, y, limit):
        magnitude = math.hypot(x, y)
        scale = min(1.0, limit / magnitude) if magnitude else 1.0
        return x * scale, y * scale, scale < 1.0

    def test_mapping_does_not_publish_unsolicited_zero(self):
        """Healthy mapping leaves the timer quiet and forwards guarded Nav2 output."""
        self.timer(self.node)
        self.node._pub.publish.assert_not_called()
        self.node._write_localization_stop.assert_called_once_with(False)
        self.command(self.node, self._command())
        self.node._pub.publish.assert_called_once()

    def test_mapping_blocks_nav2_output_when_sensor_inputs_are_stale(self):
        """Mapping heartbeat alone cannot authorize output without live inputs."""
        self.inputs_ready = False
        self.command(self.node, self._command())
        self.node._pub.publish.assert_called_once()
        self.node._cancel_for_localization.assert_called_once()

    def test_active_mapping_stops_without_another_command_when_inputs_expire(self):
        """The timer stops an existing goal even if Nav2 stops publishing commands."""
        self.inputs_ready = False
        self.node._active_uuid = 'active-goal'
        self.node._last_cmd = (0.1, 0.0)
        self.timer(self.node)
        self.node._pub.publish.assert_called_once()
        self.node._cancel_for_localization.assert_called_once()
        self.node._write_localization_stop.assert_called_once_with(True)

    def test_rotation_latch_still_stops_active_mapping_navigation(self):
        """Idle manual-mapping compatibility cannot suppress an active safety latch."""
        self.node._active_uuid = 'active-goal'
        self.node.guard.latched_reason = 'rotation limit'
        self.timer(self.node)
        self.node._pub.publish.assert_called_once()

    def test_transition_and_lost_localization_still_stop(self):
        """Missing readiness or localization still stops without incoming commands."""
        self.mapping = False
        self.node._localization_check = lambda: (False, 'not localized')
        self.timer(self.node)
        self.node._pub.publish.assert_called_once()
        self.node._cancel_for_localization.assert_called_once()

    def test_invalid_localization_still_blocks_velocity(self):
        """A localization-mode verification failure remains a hard stop."""
        self.mapping = False
        self.node._localization_check = lambda: (False, 'not localized')
        self.command(self.node, self._command())
        self.node._pub.publish.assert_called_once()
        self.node._cancel_for_localization.assert_called_once()

    def test_old_rotation_latch_does_not_fight_manual_mapping(self):
        """A previous navigation failure cannot leave a timer braking mapping."""
        self.node.guard.latched_reason = 'rotation limit'
        self.timer(self.node)
        self.node._pub.publish.assert_not_called()

    def test_verified_localization_leaves_timer_quiet(self):
        """A verified load forwards command handling without unsolicited stops."""
        self.mapping = False
        self.node._localization_check = lambda: (True, '')
        self.timer(self.node)
        self.node._pub.publish.assert_not_called()
        self.node._write_localization_stop.assert_called_once_with(False)
