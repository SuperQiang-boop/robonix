import math
import unittest

from nav2_wrapper.rotation_guard_core import GuardLimits, RotationGuard, normalize_uuid_octets


class RotationGuardTest(unittest.TestCase):
    def test_correction_pauses_only_goal_yaw_progress(self):
        """Aiming toward position during correction does not look like failed final-yaw progress."""
        guard = RotationGuard(GuardLimits(terminal_xy_m=0.2, terminal_no_progress_s=1.0))
        guard.begin_goal("goal-a")
        guard.evaluate(0.0, 0.0, 0.2, (0, 0, 0), (0.1, 0, 1.0))
        guard.set_correction_active(True)
        self.assertEqual(guard.evaluate(2.0, 0.0, 0.2, (0, 0, -0.5), (0.1, 0, 1.0)), "")
        guard.set_correction_active(False)
        self.assertEqual(guard.evaluate(2.1, 0.0, 0.2, (0, 0, -0.5), (0.1, 0, 1.0)), "")
        self.assertEqual(guard.evaluate(3.2, 0.0, 0.2, (0, 0, -0.5), (0.1, 0, 1.0)),
                         "terminal yaw made no progress")

    def test_correction_cannot_reset_outer_deadline_or_drift_radius(self):
        """Correction permission never disables the 40 cm boundary or overall terminal budget."""
        limits = GuardLimits(terminal_xy_m=0.2, terminal_timeout_s=15.0)
        for pose, now, reason in [((0.51, 0, 0), 1.0, "terminal XY drift exceeded exit tolerance"),
                                  ((0, 0, 0), 15.1, "goal vicinity timeout")]:
            guard = RotationGuard(limits)
            guard.begin_goal("goal-a")
            guard.evaluate(0.0, 0.0, 0.2, (0, 0, 0), (0.1, 0, 1.0))
            guard.set_correction_active(True)
            self.assertEqual(guard.evaluate(now, 0.1, 0.2, pose, (0.1, 0, 1.0)), reason)
            guard.begin_goal("goal-b")
            self.assertFalse(guard.correction_active)

    def test_ros_uuid_octets_are_plain_python_ints(self):
        class ByteLike:
            def __init__(self, value):
                self.value = value

            def __int__(self):
                return self.value

        result = normalize_uuid_octets(ByteLike(value) for value in range(16))
        self.assertEqual(result, list(range(16)))
        self.assertTrue(all(type(value) is int for value in result))

    def test_terminal_rotation_without_progress_latches(self):
        guard = RotationGuard(GuardLimits(terminal_no_progress_s=3.0))
        guard.begin_goal("goal-a")
        guard.observe_odom(0.0)
        self.assertEqual(guard.evaluate(0.0, 0.0, 0.2, (0, 0, 0), (0.1, 0, 1.0)), "")
        guard.observe_odom(0.2)
        reason = guard.evaluate(3.1, 0.0, 0.2, (0, 0, 0.0), (0.1, 0, 1.0))
        self.assertEqual(reason, "terminal yaw made no progress")
        self.assertEqual(guard.evaluate(3.2, 0.0, 0.0, None, None), reason)

    def test_new_action_uuid_is_the_only_reset(self):
        guard = RotationGuard(GuardLimits(terminal_no_progress_s=1.0))
        guard.begin_goal("goal-a")
        guard.evaluate(0.0, 0.0, 0.2, (0, 0, 0), (0.1, 0, 1.0))
        guard.evaluate(1.1, 0.0, 0.2, (0, 0, 0), (0.1, 0, 1.0))
        self.assertTrue(guard.latched_reason)
        guard.begin_goal("goal-a")
        self.assertTrue(guard.latched_reason)
        guard.begin_goal("goal-b")
        self.assertFalse(guard.latched_reason)

    def test_global_guard_catches_recovery_spin_away_from_goal(self):
        guard = RotationGuard(GuardLimits(global_spin_limit_rad=1.0))
        guard.begin_goal("goal-a")
        guard.observe_odom(0.0)
        guard.evaluate(0.0, 0.0, 0.2, None, None)
        guard.observe_odom(0.6)
        guard.observe_odom(1.2)
        reason = guard.evaluate(1.0, 0.0, 0.2, None, None)
        self.assertEqual(reason, "continuous stationary rotation limit")

    def test_legitimate_half_turn_is_below_global_limit(self):
        guard = RotationGuard()
        guard.begin_goal("goal-a")
        guard.observe_odom(0.0)
        guard.evaluate(0.0, 0.0, 0.2, None, None)
        for i in range(1, 17):
            guard.observe_odom(i * math.pi / 16.0)
            self.assertEqual(guard.evaluate(i * 0.5, 0.0, 0.2, None, None), "")

    def test_terminal_wandering_times_out_even_with_forward_motion(self):
        """Leaving and re-entering the near-goal window must not reset its deadline."""
        guard = RotationGuard(GuardLimits(terminal_xy_m=0.2, terminal_timeout_s=15.0))
        guard.begin_goal("goal-a")
        self.assertEqual(guard.evaluate(0.0, 0.25, 0.3, (0.1, 0, 0), (0, 0, 1)), "")
        self.assertEqual(guard.evaluate(5.0, 0.25, 0.3, (0.3, 0, 0.5), (0, 0, 1)), "")
        guard.begin_goal("goal-a")
        self.assertEqual(guard.evaluate(10.0, 0.25, 0.3, (0.1, 0, 0.5), (0, 0, 1)), "")
        self.assertEqual(
            guard.evaluate(15.1, 0.25, 0.3, (0.3, 0, 0.5), (0, 0, 1)),
            "goal vicinity timeout",
        )
        self.assertTrue(guard.latched_reason)

    def test_terminal_large_xy_excursion_stops_instead_of_chasing_again(self):
        """Stop an orbit beyond the terminal exit radius regardless of its speed."""
        guard = RotationGuard(GuardLimits(terminal_xy_m=0.2))
        guard.begin_goal("goal-a")
        guard.evaluate(0.0, 0.25, 0.3, (0.1, 0, 0), (0, 0, 1))
        self.assertEqual(
            guard.evaluate(2.0, 0.25, 0.3, (0.41, 0, 0.5), (0, 0, 1)),
            "terminal XY drift exceeded exit tolerance",
        )

    def test_long_transit_does_not_start_terminal_deadline(self):
        """Apply the new deadline only after reaching the configured terminal window."""
        guard = RotationGuard(GuardLimits(terminal_xy_m=0.2))
        guard.begin_goal("goal-a")
        self.assertEqual(guard.evaluate(0.0, 0.25, 0.1, (2, 0, 0), (0, 0, 1)), "")
        self.assertEqual(guard.evaluate(60.0, 0.25, 0.1, (0.3, 0, 0), (0, 0, 1)), "")
        self.assertIsNone(guard.approach_started_at)

    def test_new_goal_and_idle_clear_terminal_wandering_budget(self):
        """A different goal gets a fresh terminal budget, but repeated plans do not."""
        guard = RotationGuard(GuardLimits(terminal_xy_m=0.2))
        guard.begin_goal("goal-a")
        guard.evaluate(0.0, 0.25, 0.3, (0.1, 0, 0), (0, 0, 1))
        guard.begin_goal("goal-b")
        self.assertIsNone(guard.approach_started_at)
        self.assertEqual(guard.evaluate(60.0, 0.25, 0.3, (0.1, 0, 0), (0, 0, 1)), "")
        guard.clear_if_idle()
        self.assertIsNone(guard.approach_started_at)
        self.assertEqual(guard.evaluate(90.0, 0.0, 0.0, (0.1, 0, 0), (0, 0, 1)), "")

    def test_legitimate_terminal_heading_adjustment_is_allowed(self):
        """Allow heading convergence and modest bipedal sway inside the exit radius."""
        guard = RotationGuard(GuardLimits(terminal_xy_m=0.2))
        guard.begin_goal("goal-a")
        for step in range(11):
            yaw = step * 0.1
            guard.observe_odom(yaw)
            distance = 0.19 if step == 0 else 0.22
            self.assertEqual(
                guard.evaluate(step * 0.5, 0.0, 0.3, (distance, 0, yaw), (0, 0, 1)), ""
            )


if __name__ == "__main__":
    unittest.main()
