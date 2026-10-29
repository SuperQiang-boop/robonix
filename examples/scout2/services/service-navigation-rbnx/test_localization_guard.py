"""Run the actual velocity guard methods without starting ROS or a chassis."""
import ast
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
        namespace = dict(Twist=SimpleNamespace)
        exec(compile(ast.Module(body=methods, type_ignores=[]), str(path), 'exec'), namespace)
        self.timer, self.command = namespace['_publish_latched_zero'], namespace['_on_cmd']
        self.mapping = True
        module = ModuleType('relocalization_rbnx.navigation')
        module.mapping_active = lambda directory: self.mapping
        patcher = patch.dict('sys.modules', {'relocalization_rbnx.navigation': module})
        patcher.start()
        self.addCleanup(patcher.stop)
        self.node = SimpleNamespace(_lock=threading.Lock(), _localization_dir='/fixture',
            _localization_check=lambda: (False, 'not localized'), _pub=Mock(),
            _cancel_for_localization=Mock(), _write_localization_stop=Mock(),
            guard=SimpleNamespace(latched_reason=''))

    def test_mapping_does_not_publish_unsolicited_zero(self):
        """Steady mapping leaves teleoperation free while rejecting Nav2 commands."""
        self.timer(self.node)
        self.node._pub.publish.assert_not_called()
        self.node._write_localization_stop.assert_called_once_with(True)
        self.command(self.node, SimpleNamespace())
        self.node._pub.publish.assert_called_once()

    def test_transition_and_lost_localization_still_stop(self):
        """Missing readiness or localization still stops without incoming commands."""
        self.mapping = False
        self.timer(self.node)
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
