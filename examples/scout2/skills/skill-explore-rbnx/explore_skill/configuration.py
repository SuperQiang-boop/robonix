# SPDX-License-Identifier: MulanPSL-2.0
"""Validated deployment defaults and explicit per-request exploration limits."""
from dataclasses import dataclass, fields
import math


@dataclass(frozen=True)
class ExploreSettings:
    timeout_s: float = 0.0
    nav_goal_timeout_s: float = 180.0
    nav_goal_min_timeout_s: float = 30.0
    nav_time_factor: float = 1.5
    nav_turn_allowance_s: float = 30.0
    sweep_every_n_legs: int = 0
    arrival_tolerance_m: float = 0.45
    arrival_settle_s: float = 2.0
    nav_stall_timeout_s: float = 20.0

    @classmethod
    def parse(cls, config):
        """Reject invalid defaults at INIT without allocating ROS resources."""
        config = dict(config or {})
        mode = config.pop('explore_mode', 'frontier')
        if mode != 'frontier':
            raise ValueError('explore_mode must be frontier')
        unknown = config.keys() - {field.name for field in fields(cls)}
        if unknown:
            raise ValueError(f'unknown explore config fields: {sorted(unknown)}')
        settings = cls(**config)
        for field in fields(cls):
            value = getattr(settings, field.name)
            if type(value) not in (int, float) or not math.isfinite(value) or value < 0:
                raise ValueError(f'{field.name} must be finite and nonnegative')
            if value == 0 and field.name not in ('timeout_s', 'sweep_every_n_legs'):
                raise ValueError(f'{field.name} must be positive')
        if type(settings.sweep_every_n_legs) is not int:
            raise ValueError('sweep_every_n_legs must be an integer')
        if settings.nav_time_factor < 1:
            raise ValueError('nav_time_factor must be >= 1')
        if settings.nav_goal_min_timeout_s > settings.nav_goal_timeout_s:
            raise ValueError('nav_goal_min_timeout_s exceeds nav_goal_timeout_s')
        if settings.nav_turn_allowance_s >= settings.nav_goal_timeout_s:
            raise ValueError('nav_turn_allowance_s must be below nav_goal_timeout_s')
        if settings.arrival_tolerance_m > 0.5:
            raise ValueError('arrival_tolerance_m must be <= 0.5')
        return settings

    def resolve_timeout(self, requested_s):
        """-1 selects the deployment default; 0 means unlimited; positive overrides."""
        if type(requested_s) not in (int, float) or not math.isfinite(requested_s):
            raise ValueError('timeout_s must be finite')
        if requested_s == -1:
            return float(self.timeout_s)
        if requested_s < 0:
            raise ValueError('timeout_s must be -1, 0, or positive')
        return float(requested_s)
