# SPDX-License-Identifier: MulanPSL-2.0
"""The velocity actually sent to the base has to be readable from nav2.log.

On the 2026-08-24 deployment the controller aborted sixteen times with "Failed
to make progress" and the logs could not answer whether nav2 had commanded any
motion at all: every command was in the per-goal trace directory, which was not
collected. These tests pin the summary that puts the answer in the log itself.
"""
from __future__ import annotations

import unittest

from nav2_wrapper.cmd_vel_summary import CmdVelSummary


class CmdVelSummaryTest(unittest.TestCase):
    def test_the_first_command_opens_the_window_rather_than_logging(self):
        s = CmdVelSummary(period_s=1.0)
        self.assertIsNone(s.observe(100.0, 0.1, 0.0, False, False))

    def test_a_stalled_robot_reads_as_every_command_below_the_threshold(self):
        s = CmdVelSummary(period_s=1.0)
        s.observe(100.0, 0.0, 0.4, False, False)
        lines = [s.observe(100.0 + i * 0.05, 0.0, 0.4, False, False)
                 for i in range(1, 25)]
        emitted = [l for l in lines if l]
        self.assertEqual(len(emitted), 1, lines)
        line = emitted[0]
        # 21 commands land inside the window: the one that opened it plus the
        # twenty that followed before the period elapsed.
        self.assertIn("window n=21", line)
        self.assertIn("21 below", line)
        self.assertIn("peak linear=0.000", line)
        # The angular peak separates "commanded nothing" from "commanded a
        # rotation the base did not execute" -- different faults, same symptom.
        self.assertIn("angular=0.400", line)

    def test_a_moving_robot_reports_its_peak_linear(self):
        s = CmdVelSummary(period_s=1.0)
        s.observe(100.0, 0.05, 0.0, False, False)
        line = s.observe(101.5, 0.25, 0.0, False, False)
        self.assertIn("peak linear=0.250", line)

    def test_nothing_is_emitted_before_the_period_elapses(self):
        s = CmdVelSummary(period_s=1.0)
        s.observe(100.0, 0.1, 0.0, False, False)
        lines = [s.observe(100.0 + i * 0.01, 0.1, 0.0, False, False)
                 for i in range(1, 50)]
        self.assertEqual([l for l in lines if l], [])

    def test_counters_reset_between_windows(self):
        s = CmdVelSummary(period_s=1.0)
        s.observe(100.0, 0.4, 0.0, False, False)
        s.observe(101.1, 0.4, 0.0, False, False)
        second = s.observe(102.2, 0.0, 0.0, False, False)
        self.assertIn("window n=1", second)
        self.assertIn("peak linear=0.000", second)

    def test_limit_and_guard_state_travel_with_the_line(self):
        s = CmdVelSummary(period_s=1.0)
        s.observe(100.0, 0.3, 0.1, True, True)
        line = s.observe(101.1, 0.3, 0.1, True, True)
        self.assertIn("limited=True", line)
        self.assertIn("latched=True", line)

    def test_the_still_threshold_is_configurable(self):
        s = CmdVelSummary(period_s=1.0, still_linear_mps=0.5)
        s.observe(100.0, 0.3, 0.0, False, False)
        line = s.observe(101.1, 0.3, 0.0, False, False)
        self.assertIn("2 below 0.50 m/s", line)


if __name__ == "__main__":
    unittest.main()
