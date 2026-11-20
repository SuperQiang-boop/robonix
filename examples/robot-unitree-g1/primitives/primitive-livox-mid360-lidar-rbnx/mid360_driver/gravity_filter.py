"""Timestamped gravity estimation; acceleration is SI, angular velocity rad/s."""

from collections import deque
import math

import numpy as np
from scipy.spatial.transform import Rotation


def level_rotation(up):
    """Return the shortest rotation from measured up to +Z, without estimating yaw."""
    up = np.asarray(up, dtype=float)
    up = up / np.linalg.norm(up)
    if not np.isfinite(up).all() or up[2] < 0.5:
        raise ValueError("gravity outside the supported upright envelope")
    v = np.cross(up, [0.0, 0.0, 1.0])
    skew = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
    return np.eye(3) + skew + skew @ skew / (1.0 + up[2])


class GravityFilter:
    """Fuse gyro propagation with gated, slow accelerometer correction.

    Initialize only after one stationary second. Missing IMU for 0.1 s resets
    initialization; no orientation is returned across a gap or outside history.
    Linear acceleration close to gravity cannot be fully distinguished by a
    six-axis IMU; slow correction limits its effect but needs walking validation.
    """

    def __init__(self):
        """Keep three seconds of estimates and a bounded stationary warmup."""
        self.history = deque()
        self.warmup = []
        self.up = None
        self.bias = np.zeros(3)
        self.last_stamp = None

    def update(self, stamp, acceleration, gyro):
        """Accept an ordered IMU sample, resetting on gaps and clock rewinds."""
        acceleration, gyro = np.asarray(acceleration), np.asarray(gyro)
        if not (np.isfinite(acceleration).all() and np.isfinite(gyro).all()):
            return
        if self.last_stamp is not None:
            dt = stamp - self.last_stamp
            if -0.5 < dt <= 0:
                return
            if dt <= -0.5 or dt > 0.1:
                self.history.clear()
                self.warmup.clear()
                self.up = None
        else:
            dt = 0.0
        self.last_stamp = stamp
        magnitude = np.linalg.norm(acceleration)
        valid_accel = abs(magnitude - 9.80665) < 0.08 * 9.80665
        if self.up is None:
            if not valid_accel or np.linalg.norm(gyro) > 0.04:
                self.warmup.clear()
                return
            self.warmup.append((stamp, acceleration.copy(), gyro.copy()))
            if stamp - self.warmup[0][0] < 1.0 or len(self.warmup) < 100:
                return
            accelerations = np.array([item[1] for item in self.warmup])
            if np.max(np.std(accelerations, axis=0)) > 0.02 * 9.80665:
                self.warmup.clear()
                return
            self.up = np.mean(accelerations, axis=0)
            self.up /= np.linalg.norm(self.up)
            self.bias = np.mean([item[2] for item in self.warmup], axis=0)
            self.warmup.clear()
        else:
            self.up = Rotation.from_rotvec(-(gyro - self.bias) * dt).apply(self.up)
            if valid_accel:
                measured = acceleration / magnitude
                # Large innovations are usually translation, not actual tilt:
                # actual tilt is already followed immediately by the gyro.
                if measured @ self.up > math.cos(math.radians(5.0)):
                    alpha = 1.0 - math.exp(-dt / 2.0)
                    self.up = (1.0 - alpha) * self.up + alpha * measured
            self.up /= np.linalg.norm(self.up)
        self.history.append((stamp, self.up.copy()))
        while self.history and stamp - self.history[0][0] > 3.0:
            self.history.popleft()

    def at(self, stamp):
        """Interpolate bracketing samples; never use a future/latest fallback."""
        if not self.history or stamp < self.history[0][0] or stamp > self.history[-1][0]:
            return None
        for index in range(len(self.history) - 1, -1, -1):
            before_stamp, before = self.history[index]
            if before_stamp <= stamp:
                if before_stamp == stamp:
                    return before.copy()
                after_stamp, after = self.history[index + 1]
                weight = (stamp - before_stamp) / (after_stamp - before_stamp)
                up = before * (1.0 - weight) + after * weight
                return up / np.linalg.norm(up)
        return None
