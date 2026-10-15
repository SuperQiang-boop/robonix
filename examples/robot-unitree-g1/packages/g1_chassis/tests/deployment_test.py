"""Offline checks for external-odometry ownership and the conservative Nav2 profile."""
import importlib.util
import math
from pathlib import Path
import sys
from types import ModuleType, SimpleNamespace
import unittest
from unittest.mock import patch

import yaml

ROOT = Path(__file__).resolve().parents[3]


def load_module(name, path):
    """Load package-local helpers without starting Robonix or ROS processes."""
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class DeploymentTest(unittest.TestCase):
    def setUp(self):
        """Read the active deployment and keep provider imports offline with inert decorators."""
        self.manifest = yaml.safe_load((ROOT / "robonix_manifest.yaml").read_text())
        entries = [entry for group in self.manifest.values() if isinstance(group, list)
                   for entry in group if isinstance(entry, dict) and "config" in entry]
        self.configs = {entry["name"]: entry["config"] for entry in entries}
        self.params = yaml.safe_load((ROOT / "config/nav2_params_g1.yaml").read_text())
        api = ModuleType("robonix_api")
        api.Primitive = lambda **kwargs: SimpleNamespace(on_init=lambda fn: fn, on_shutdown=lambda fn: fn)
        api.Deferred = api.Err = api.Ok = lambda *args: args
        with patch.dict(sys.modules, {"robonix_api": api}):
            self.provider = load_module("g1_test_provider", ROOT / "packages/g1_chassis/g1_chassis/main.py")

    def test_external_odometry_is_explicitly_selected(self):
        """Verify mapping cannot accidentally fall back to its internal ICP odometry."""
        mapping = self.configs["mapping"]
        profiles = load_module("g1_test_profiles", ROOT / "services/service-map-rbnx/src/mapping_rbnx/profiles.py")
        selected = profiles.select_rtabmap_inputs(mapping["rtabmap_inputs"], {
            "lidar_topic": "/livox/lidar", "imu_topic": "/livox/imu", "odom_topic": "/odom",
        })
        self.assertEqual(selected["odom_topic"], "/odom")
        self.assertEqual(mapping["sensor_providers"]["odom"], "g1_chassis")
        self.assertTrue(self.configs["g1_chassis"]["publish_odom_tf"])
        nav = self.configs["nav2"]
        self.assertEqual(nav["provider_ids"]["odom"], "g1_chassis")
        self.assertEqual(nav["topic_remap"]["odom"], "/odom")

    def test_mppi_timing_and_speed_envelope(self):
        """Verify controller sampling matches its loop and never expands chassis limits."""
        controller = self.params["controller_server"]["ros__parameters"]
        mppi = controller["FollowPath"]
        chassis = self.configs["g1_chassis"]
        self.assertEqual(mppi["motion_model"], "DiffDrive")
        self.assertAlmostEqual(mppi["model_dt"], 1 / controller["controller_frequency"])
        self.assertLessEqual(mppi["vx_max"], chassis["max_linear_x_mps"])
        self.assertLessEqual(mppi["wz_max"], chassis["max_angular_z_rps"])
        self.assertEqual(mppi["vy_max"], 0)
        self.assertEqual(mppi["vx_min"], 0)
        self.assertEqual(self.params["planner_server"]["ros__parameters"]["GridBased"]["plugin"],
                         "nav2_smac_planner/SmacPlanner2D")
        self.assertLessEqual(chassis["state_timeout_s"], mppi["transform_tolerance"])

    def test_rotation_shim_owns_terminal_heading_without_expanding_limits(self):
        """Keep the MPPI deadband out of pure heading adjustments and preserve goal tolerances."""
        controller = self.params["controller_server"]["ros__parameters"]
        follow_path = controller["FollowPath"]
        self.assertEqual(follow_path["plugin"],
                         "robonix_nav2_terminal::PersistentRotationShim")
        self.assertEqual(follow_path["primary_controller"], "nav2_mppi_controller::MPPIController")
        self.assertTrue(follow_path["rotate_to_goal_heading"])
        self.assertLessEqual(follow_path["rotate_to_heading_angular_vel"], follow_path["wz_max"])
        self.assertFalse(follow_path["closed_loop"])
        self.assertGreater(follow_path["max_angular_accel"], 0.0)
        self.assertLess(follow_path["angular_disengage_threshold"],
                        follow_path["angular_dist_threshold"])
        checker = controller["general_goal_checker"]
        self.assertEqual(checker["xy_goal_tolerance"], 0.20)
        self.assertEqual(checker["yaw_goal_tolerance"], 0.25)
        self.assertEqual(checker["plugin"], "robonix_nav2_terminal::SettledGoalChecker")
        self.assertEqual(checker["settle_time"], follow_path["terminal_settle_time"])
        self.assertEqual(checker["trans_stopped_velocity"],
                         follow_path["terminal_stopped_linear_velocity"])
        self.assertEqual(checker["rot_stopped_velocity"],
                         follow_path["terminal_stopped_angular_velocity"])
        self.assertEqual(follow_path["terminal_timeout"],
                         self.configs["nav2"]["guard_terminal_timeout_s"])
        self.assertEqual(follow_path["terminal_xy_exit_tolerance"],
                         2 * checker["xy_goal_tolerance"])
        self.assertLess(follow_path["terminal_correction_xy_tolerance"], checker["xy_goal_tolerance"])
        self.assertGreater(follow_path["terminal_correction_trigger"], checker["xy_goal_tolerance"])
        self.assertLess(follow_path["terminal_correction_trigger"], follow_path["terminal_xy_exit_tolerance"])
        self.assertLessEqual(follow_path["terminal_correction_max_speed"], follow_path["vx_max"])
        self.assertLess(follow_path["terminal_correction_timeout"], follow_path["terminal_timeout"])
        self.assertEqual(self.configs["nav2"]["guard_terminal_xy_m"], checker["xy_goal_tolerance"])

    def test_mppi_deadband_penalty_preserves_stop_and_collision_constraints(self):
        """Avoid suspected ineffective forward speeds without flooring actuator commands."""
        controller = self.params["controller_server"]["ros__parameters"]
        mppi = controller["FollowPath"]
        self.assertIn("VelocityDeadbandCritic", mppi["critics"])
        deadband = mppi["VelocityDeadbandCritic"]
        self.assertTrue(deadband["enabled"])
        self.assertEqual(deadband["cost_power"], 1)
        self.assertEqual(deadband["cost_weight"], 35.0)
        self.assertEqual(deadband["deadband_velocities"], [0.20, 0.0, 0.0])
        self.assertLess(deadband["deadband_velocities"][0], mppi["vx_max"])
        self.assertEqual(mppi["vx_min"], 0.0)
        self.assertIn("ConstraintCritic", mppi["critics"])
        self.assertIn("CostCritic", mppi["critics"])
        self.assertTrue(mppi["CostCritic"]["consider_footprint"])
        self.assertEqual(controller["general_goal_checker"]["xy_goal_tolerance"], 0.20)
        self.assertEqual(controller["progress_checker"]["movement_time_allowance"], 30.0)

    def test_provider_config_rejects_invalid_inputs(self):
        """Check bad source, timing and non-finite limits fail before process launch."""
        self.provider._validate_config({})
        for invalid in ({"odom_source": "placeholder"}, {"state_topic": ""},
                        {"odom_publish_rate_hz": 0}, {"state_timeout_s": 1},
                        {"max_linear_x_mps": math.nan}, {"max_angular_z_rps": -1},
                        {"state_topic": "rt/a b"}, {"publish_odom_tf": "unknown"}):
            with self.subTest(invalid=invalid), self.assertRaises(ValueError):
                self.provider._validate_config(invalid)

    def test_provider_arguments_preserve_no_motion_default(self):
        """Ensure state subscription does not implicitly enable SDK motion."""
        with patch.dict("os.environ", {"G1_ALLOW_MOTION": "false"}):
            argv = self.provider._daemon_argv("/tmp/g1-test.sock", {"network_interface": "lo"})
        self.assertIn("rt/odommodestate", argv)
        self.assertNotIn("--allow-motion", argv)
        self.assertEqual(argv[argv.index("--max-vx") + 1], "0.35")
        self.assertIn("use_sim_time:=false", self.provider._adapter_argv({}))

    def test_timing_parameters_are_ros_double_values(self):
        """Accept numeric YAML integers without changing the ROS parameter's declared type."""
        argv = self.provider._adapter_argv({"odom_publish_rate_hz": 50, "state_timeout_s": "0.15"})
        self.assertIn("odom_publish_rate_hz:=50.0", argv)
        self.assertIn("state_timeout_s:=0.15", argv)

    def test_invalid_boolean_never_enables_motion(self):
        """Reject unresolved environment placeholders instead of silently dropping a gate."""
        with self.assertRaises(ValueError):
            self.provider._daemon_argv("/tmp/g1-test.sock", {
                "network_interface": "lo", "allow_motion": "${G1_ALLOW_MOTION}",
            })

    def test_adapter_start_failure_cleans_up_daemon(self):
        """Check partial startup failure returns diagnostics and rolls back owned processes."""
        daemon = SimpleNamespace(poll=lambda: None)
        with patch.dict("os.environ", {"G1_ALLOW_MOTION": "false", "G1_IPC_SOCKET": "/tmp/g1-test.sock"}), \
                patch.object(self.provider, "_is_executable", return_value=True), \
                patch.object(self.provider.os, "makedirs"), \
                patch.object(self.provider.os.path, "exists", return_value=True), \
                patch.object(self.provider, "_stop_processes", side_effect=self.provider._processes.clear) as stop, \
                patch.object(self.provider.g1_chassis, "spawn", side_effect=[daemon, OSError("adapter missing")], create=True):
            result = self.provider.initialize({"network_interface": "lo"})
        self.assertIn("adapter missing", result[0])
        self.assertEqual(stop.call_count, 2)
        self.assertFalse(self.provider._processes)

    def test_no_state_does_not_declare_ready_contracts(self):
        """A discovered publisher without a real odometry message must not pass readiness."""
        process = SimpleNamespace(poll=lambda: None)
        with patch.dict("os.environ", {"G1_ALLOW_MOTION": "false", "G1_IPC_SOCKET": "/tmp/g1-test.sock"}), \
                patch.object(self.provider, "_is_executable", return_value=True), \
                patch.object(self.provider.os, "makedirs"), \
                patch.object(self.provider.os.path, "exists", return_value=True), \
                patch.object(self.provider, "_stop_processes", side_effect=self.provider._processes.clear), \
                patch.object(self.provider.g1_chassis, "spawn", return_value=process, create=True), \
                patch.object(self.provider.g1_chassis, "wait_for_topic", return_value=False, create=True), \
                patch.object(self.provider.g1_chassis, "declare_ros2_topic", create=True) as declare:
            result = self.provider.initialize({"network_interface": "lo"})
        self.assertIn("No real G1 odometry", result[0])
        declare.assert_not_called()
        self.assertFalse(self.provider._processes)


if __name__ == "__main__":
    unittest.main()
