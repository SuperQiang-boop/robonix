"""Offline algorithm, private-worker and deployment checks; no ROS or robot motion."""
import importlib.util
import math
from pathlib import Path
import subprocess
import sys
import unittest
import xml.etree.ElementTree as ET

import yaml

ROOT = Path(__file__).resolve().parent
SCRIPT = ROOT / 'terminal_controller/python/final_approach_g1.py'
spec = importlib.util.spec_from_file_location('final_approach_g1', SCRIPT)
module = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = module
spec.loader.exec_module(module)


class FinalApproachTest(unittest.TestCase):
    def test_body_frame_sideways_and_reverse_preserve_original_gait_floors(self):
        """A goal to the left/back commands G1 lateral/reverse motion, not a detour turn."""
        for goal, expected in [((0, .3, 0), (0, .02, 0)),
                               ((-.3, 0, 0), (-.025, 0, 0))]:
            control = module.FinalApproachController()
            command = control.step((0, 0, 0), goal, 1, .35)
            for actual, value in zip(command[:3], expected):
                self.assertAlmostEqual(actual, value)
            self.assertFalse(command[3])
        control = module.FinalApproachController()
        command = control.step((0, 0, math.pi / 2), (.3, 0, math.pi / 2), 1, .35)
        self.assertAlmostEqual(command[1], -.02)

    def test_pose_dwell_resets_on_leaving_original_eight_centimetre_window(self):
        """Original completion needs continuous pose acceptance, not XY entry latching."""
        control = module.FinalApproachController(module.Parameters(filter_alpha=1))
        self.assertFalse(control.step((0, 0, 0), (0, 0, 0), 1, .35)[3])
        self.assertFalse(control.step((.2, 0, 0), (0, 0, 0), 1.9, .35)[3])
        self.assertFalse(control.step((0, 0, 0), (0, 0, 0), 2, .35)[3])
        self.assertFalse(control.step((0, 0, 0), (0, 0, 0), 2.99, .35)[3])
        self.assertEqual(control.step((0, 0, 0), (0, 0, 0), 3.01, .35), (0, 0, 0, True))

    def test_speed_limit_overrides_gait_floor_and_accumulated_speed(self):
        """Floors cannot defeat external speed caps, even after commands ramp up."""
        control = module.FinalApproachController()
        for i in range(20):
            vx, vy, w, _ = control.step((0, 0, 0), (.4, .4, 1), i / 20, .10)
            self.assertLessEqual(math.hypot(vx, vy), .10000001)
            self.assertLessEqual(abs(w), .50)
        self.assertEqual(control.step((0, 0, 0), (.4, .4, 1), 2, 0)[:2], (0, 0))

    def test_angle_filter_wraps_across_pi(self):
        control = module.FinalApproachController()
        control.step((0, 0, math.pi - .01), (0, 0, math.pi), 1, .35)
        control.step((0, 0, -math.pi + .01), (0, 0, math.pi), 1.05, .35)
        self.assertLess(abs(module.normalize(control.filtered[2] - math.pi)), .02)

    def test_real_worker_protocol_reset_and_invalid_input(self):
        """Exercise the actual Python subprocess and line protocol used by the adapter."""
        result = subprocess.run([sys.executable, str(SCRIPT)], input=(
            '0 0 0 0 0 0 1 .35 1\n'
            '0 0 0 0 0 0 2.1 .35 0\n'
            '0 0 0 0 0 0 3 .35 1\n'),
            capture_output=True, text=True, timeout=5, check=True)
        lines = [list(map(float, line.split())) for line in result.stdout.splitlines()]
        self.assertEqual([line[3] for line in lines], [0, 1, 0])
        bad = subprocess.run([sys.executable, str(SCRIPT)], input='nan 0 0 0 0 0 1 .35 1\n',
                             capture_output=True, text=True, timeout=5)
        self.assertNotEqual(bad.returncode, 0)
        self.assertEqual(bad.stdout, '')

    def test_profiles_select_independent_plugins_without_changing_transit_or_maps(self):
        """Keep the original profile selected by default and both pipelines mutually exclusive."""
        deploy = ROOT.parents[1]
        original = yaml.safe_load((deploy / 'config/nav2_params_g1.yaml').read_text())
        alternate = yaml.safe_load((deploy / 'config/nav2_params_g1_python.yaml').read_text())
        self.assertEqual(set(original), set(alternate))
        for key in original:
            if key != 'controller_server':
                self.assertEqual(original[key], alternate[key])
        params = alternate['controller_server']['ros__parameters']
        original_params = original['controller_server']['ros__parameters']
        self.assertEqual(set(original_params), set(params))
        for key in original_params:
            if key not in ('FollowPath', 'general_goal_checker'):
                self.assertEqual(original_params[key], params[key], key)
        follow = params['FollowPath']
        self.assertEqual(follow['plugin'], 'robonix_nav2_terminal::PythonFinalApproachController')
        self.assertEqual(params['general_goal_checker']['plugin'],
                         'robonix_nav2_terminal::PythonFinalApproachGoalChecker')
        self.assertEqual(params['controller_frequency'], 20)
        shim_only = {'primary_controller', 'rotate_to_goal_heading',
                     'angular_dist_threshold', 'angular_disengage_threshold',
                     'forward_sampling_distance', 'rotate_to_heading_angular_vel',
                     'max_angular_accel', 'simulate_ahead_time', 'closed_loop'}
        for key, value in original_params['FollowPath'].items():
            if key != 'plugin' and key not in shim_only and not key.startswith('terminal_'):
                self.assertIn(key, follow, key)
                self.assertEqual(value, follow[key], key)
        self.assertEqual(follow['dist_tolerance_enter'], .08)
        self.assertEqual(follow['takeover_dist'], .60)
        xml = ET.fromstring('<plugins>' + (ROOT / 'terminal_controller/nav2_core_plugins.xml').read_text() + '</plugins>')
        classes = {item.attrib['name'] for item in xml.iter('class')}
        self.assertIn(follow['plugin'], classes)
        self.assertIn(params['general_goal_checker']['plugin'], classes)
        manifest = yaml.safe_load((deploy / 'robonix_manifest.yaml').read_text())
        nav = next(service for service in manifest['service'] if service['name'] == 'nav2')
        self.assertEqual(nav['config']['params_file'], 'config/nav2_params_g1.yaml')


if __name__ == '__main__':
    unittest.main()
