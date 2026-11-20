#!/usr/bin/env python3
"""G1 final approach algorithm ported from /data/final_approach_g1.py.

Nav2 supplies pose/goal/time over a private stdin/stdout pipe. ROS topics, TF,
action ownership and ordinary path following stay in the Nav2 adapter. No
costmap or obstacle logic is added here. Units are metres, radians and seconds.
"""
import argparse
import math
import sys
from dataclasses import dataclass


@dataclass
class Parameters:
    Kp_linear: float = 0.35
    Kp_lateral: float = 0.30
    Kp_angular: float = 0.35
    max_linear_vel: float = 0.35
    max_lateral_vel: float = 0.20
    max_angular_vel: float = 0.50
    enable_gait_min_vel: float = 1.0
    gait_min_linear_vel: float = 0.20
    gait_min_lateral_vel: float = 0.20
    gait_min_angular_vel: float = math.radians(10.0)
    dist_tolerance_enter: float = 0.08
    dist_tolerance_exit: float = 0.10
    angle_tolerance: float = 0.12
    max_dv: float = 0.025
    max_dvy: float = 0.020
    max_dw: float = 0.040
    stable_time_required: float = 1.0
    filter_alpha: float = 0.30
    align_speed_scale: float = 0.50


def normalize(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def clamp(value, limit):
    return max(-limit, min(limit, value))


class FinalApproachController:
    def __init__(self, params=None):
        """Keep the original controller's filtering, hysteresis and command history."""
        self.params = params or Parameters()
        self.filtered = None
        self.last = (0.0, 0.0, 0.0)
        self.aligning_heading = False
        self.stable_start_time = None
        self.task_done = False

    def step(self, pose, goal, now, speed_cap):
        """Compute one original 20 Hz body-frame control step; return vx/vy/w/done.

        Completion retains the uploaded script's pose-in-range dwell, not an
        additional measured-speed test. External Nav2 speed limits cap planar
        speed after gait floors, so the floors cannot override a speed limit.
        """
        p = self.params
        if self.task_done:
            return 0.0, 0.0, 0.0, True
        if self.filtered is None:
            self.filtered = pose
        else:
            x, y, yaw = self.filtered
            a = p.filter_alpha
            self.filtered = (x + a * (pose[0] - x), y + a * (pose[1] - y),
                             normalize(yaw + a * normalize(pose[2] - yaw)))
        x, y, yaw = self.filtered
        dx, dy = goal[0] - x, goal[1] - y
        distance = math.hypot(dx, dy)
        error = normalize(goal[2] - yaw)
        if distance <= p.dist_tolerance_enter:
            self.aligning_heading = True
        elif distance > p.dist_tolerance_exit:
            self.aligning_heading = False
        reached = distance <= p.dist_tolerance_enter and abs(error) <= p.angle_tolerance
        if reached:
            if self.stable_start_time is None or now < self.stable_start_time:
                self.stable_start_time = now
            elif now - self.stable_start_time >= p.stable_time_required:
                self.task_done = True
                self.last = (0.0, 0.0, 0.0)
                return 0.0, 0.0, 0.0, True
        else:
            self.stable_start_time = None
        vx = p.Kp_linear * (dx * math.cos(yaw) + dy * math.sin(yaw))
        vy = p.Kp_lateral * (-dx * math.sin(yaw) + dy * math.cos(yaw))
        w = p.Kp_angular * error
        if self.aligning_heading:
            vx *= p.align_speed_scale
            vy *= p.align_speed_scale
        targets = [vx, vy, w] if not reached else [0.0, 0.0, 0.0]
        limits = (p.max_linear_vel, p.max_lateral_vel, p.max_angular_vel)
        floors = (p.gait_min_linear_vel, p.gait_min_lateral_vel, p.gait_min_angular_vel)
        for i in range(3):
            targets[i] = clamp(targets[i], limits[i])
            if not reached and p.enable_gait_min_vel and 1e-6 < abs(targets[i]) < floors[i]:
                targets[i] = math.copysign(floors[i], targets[i])
            targets[i] = clamp(targets[i], limits[i])
        deltas = (p.max_dv, p.max_dvy, p.max_dw)
        command = [last + clamp(target - last, delta)
                   for target, last, delta in zip(targets, self.last, deltas)]
        speed = math.hypot(*command[:2])
        if speed > speed_cap:
            command[0] *= speed_cap / speed
            command[1] *= speed_cap / speed
        self.last = tuple(command)
        return *self.last, False


def main():
    """Serve one private Nav2 pipe; malformed/nonfinite input exits without motion."""
    parser = argparse.ArgumentParser(description=__doc__)
    for name, field in Parameters.__dataclass_fields__.items():
        parser.add_argument('--' + name, type=float, default=field.default)
    params = Parameters(**vars(parser.parse_args()))
    if any(not math.isfinite(v) or v < 0 for v in vars(params).values()):
        parser.error('parameters must be finite and nonnegative')
    if not 0 < params.filter_alpha <= 1 or params.dist_tolerance_exit < params.dist_tolerance_enter:
        parser.error('invalid filter alpha or distance hysteresis')
    controller = FinalApproachController(params)
    for line in sys.stdin:
        values = list(map(float, line.split()))
        if len(values) != 9 or not all(math.isfinite(v) for v in values) or values[7] < 0:
            raise ValueError('expected pose[3], goal[3], now, speed_cap, reset')
        if values[8]:
            controller = FinalApproachController(params)
        vx, vy, w, done = controller.step(values[:3], values[3:6], values[6], values[7])
        print(vx, vy, w, int(done), flush=True)


if __name__ == '__main__':
    main()
