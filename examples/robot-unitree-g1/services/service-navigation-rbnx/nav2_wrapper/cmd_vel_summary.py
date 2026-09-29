# SPDX-License-Identifier: MulanPSL-2.0
"""Rolling summary of the velocity actually sent to the base.

Every command is written to the per-goal trace file, but a trace is a separate
directory an operator has to remember to collect. When a robot will not move on
site the first question is whether nav2 asked it to, and that question has to be
answerable from the log file that gets sent back. This builds the one-per-second
line that answers it. Kept free of rclpy so it can be tested directly.
"""
from __future__ import annotations

# A command below this will not move a robot, whatever the base. Used only to
# describe outgoing velocity; nothing is gated on it.
STILL_LINEAR_MPS = 0.02


class CmdVelSummary:
    """Accumulate outgoing commands and emit a line at most once per period."""

    def __init__(self, period_s: float = 1.0,
                 still_linear_mps: float = STILL_LINEAR_MPS):
        self._period = period_s
        self._still = still_linear_mps
        self._logged_at = None
        self._reset()

    def _reset(self) -> None:
        self.count = 0
        self.still = 0
        self.peak_linear = 0.0
        self.peak_angular = 0.0

    def observe(self, now: float, linear: float, angular: float,
                limited: bool, latched: bool) -> str | None:
        """Record one command; return a log line when the period has elapsed.

        The first command starts the period rather than emitting immediately,
        so the first line still describes a full window.
        """
        self.count += 1
        if abs(linear) < self._still:
            self.still += 1
        self.peak_linear = max(self.peak_linear, abs(linear))
        self.peak_angular = max(self.peak_angular, abs(angular))
        if self._logged_at is None:
            self._logged_at = now
            return None
        if now - self._logged_at < self._period:
            return None
        line = (
            "cmd_vel out: linear=%.3f angular=%.3f | window n=%d, %d below "
            "%.2f m/s, peak linear=%.3f angular=%.3f | limited=%s latched=%s"
            % (linear, angular, self.count, self.still, self._still,
               self.peak_linear, self.peak_angular, limited, latched)
        )
        self._logged_at = now
        self._reset()
        return line
