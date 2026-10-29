"""Use Atlas-resolved mapping endpoints for pose injection and fresh feedback."""
from __future__ import annotations

from collections import deque
import copy
import math
import threading
import time

import numpy as np

from .computation import _remaining
from .observations import rotation, stamp_ns


class MappingIO:
    """Own only a mapping pose subscription and its pose_estimate gRPC channel.

    Caller owns/spins the ROS node and owns Atlas connections. Endpoints must
    come from the configured mapping provider; no topic names are inferred.
    Pose feedback is not proof of localization without independent scan checks.
    """

    def __init__(self, node, pose_endpoint, estimate_endpoint, *, map_frame='map', max_age_s=0.5,
                 require_injection=False):
        """Subscribe before injection so feedback is bounded and timestamped."""
        import grpc
        import robonix_contracts_pb2_grpc as contracts
        from geometry_msgs.msg import PoseWithCovarianceStamped
        from rclpy.qos import qos_profile_sensor_data
        if not isinstance(pose_endpoint, str) or not pose_endpoint.startswith('/'):
            raise ValueError('absolute Atlas mapping pose endpoint is required')
        if not isinstance(estimate_endpoint, str) or not estimate_endpoint.strip():
            raise ValueError('Atlas mapping pose_estimate endpoint is required')
        if not map_frame or not math.isfinite(max_age_s) or not 0 < max_age_s <= 2:
            raise ValueError('explicit map frame and bounded pose freshness are required')
        self.node, self.map_frame, self.max_age_s = node, map_frame, max_age_s
        self._condition = threading.Condition()
        self._poses = deque(maxlen=100)
        self._error, self._closed = None, False
        self._last_stamp = self._last_clock = 0
        self._feedback_enabled = not require_injection
        self._feedback_after = 0
        self._subscription = self._channel = None
        try:
            self._channel = grpc.insecure_channel(estimate_endpoint,
                options=[('grpc.enable_http_proxy', 0)])
            self._estimate = contracts.RobonixServiceMapPoseEstimateStub(self._channel).PoseEstimate
            self._subscription = node.create_subscription(PoseWithCovarianceStamped,
                pose_endpoint, self._receive, qos_profile_sensor_data)
        except Exception:
            self.close()
            raise

    def _receive(self, message):
        """Keep bounded valid feedback; latch malformed or reset pose streams."""
        with self._condition:
            if self._closed or self._error is not None:
                return
            if not self._feedback_enabled:
                return  # Before handoff, mapping may have no valid global pose.
            if (self._feedback_after and message.header.stamp.sec*1_000_000_000+
                    message.header.stamp.nanosec <= self._feedback_after):
                return  # Ignore queued pre-injection TF; never refresh its age.
            try:
                now = self.node.get_clock().now().nanoseconds
                stamp = stamp_ns(message.header.stamp)
                if now <= 0 or now < self._last_clock or stamp > now:
                    raise ValueError('mapping pose clock reset or future timestamp')
                if now-stamp > self.max_age_s*1e9 or stamp < self._last_stamp:
                    raise ValueError('mapping pose is stale or timestamps are not increasing')
                if message.header.frame_id != self.map_frame:
                    raise ValueError('mapping pose frame differs from configured map frame')
                pose = message.pose.pose
                attitude = rotation(pose.orientation)
                position = np.array([pose.position.x, pose.position.y, pose.position.z], dtype=float)
                if not np.isfinite(position).all():
                    raise ValueError('mapping pose contains nonfinite position')
                yaw = math.atan2(attitude[1, 0], attitude[0, 0])
                pitch = math.atan2(-attitude[2, 0], math.hypot(attitude[0, 0], attitude[1, 0]))
                roll = math.atan2(attitude[2, 1], attitude[2, 2])
                value = dict(stamp_ns=stamp,
                    x=float(position[0]), y=float(position[1]), z=float(position[2]),
                    yaw=yaw, roll=roll, pitch=pitch)
                if stamp == self._last_stamp:
                    previous = {name: self._poses[-1][name] for name in value}
                    if value != previous:
                        raise ValueError('mapping changed pose at the same acquisition time')
                    return  # TF timer repeats must not refresh acquisition age.
                self._poses.append(dict(value, received=time.monotonic()))
                self._last_stamp, self._last_clock = stamp, now
            except Exception as exc:
                self._error = exc
            self._condition.notify_all()

    def _check(self, deadline, cancelled):
        """Check shutdown, feedback integrity, clock and operation cancellation."""
        remaining = _remaining(deadline, cancelled)
        if self._closed:
            raise RuntimeError('mapping connection is closed')
        if self._error is not None:
            raise RuntimeError(f'mapping pose input failed: {self._error}') from self._error
        now = self.node.get_clock().now().nanoseconds
        if now <= 0 or now < self._last_clock:
            raise ValueError('mapping ROS clock reset')
        self._last_clock = now
        return remaining, now

    def inject(self, candidate, deadline, cancelled, authorize):
        """Send one pose_estimate RPC; never retry an uncertain pose injection.

        authorize() must enforce the current request/map transition interlock.
        Mapping must serialize map changes across the RPC for that interlock to
        be effective. Cancellation stops waiting but cannot retract a pose that
        mapping already published; all subsequent validation still must pass.
        """
        import grpc
        import map_pb2
        if not callable(authorize):
            raise TypeError('current request and mapping interlock is required')
        if not all(math.isfinite(value) for value in (candidate.x, candidate.y, candidate.yaw)):
            raise ValueError('candidate pose must be finite')
        with self._condition:
            self._check(deadline, cancelled)
        authorize()
        remaining = _remaining(deadline, cancelled)
        future = self._estimate.future(map_pb2.PoseEstimate_Request(x=candidate.x,
            y=candidate.y, theta=candidate.yaw, cov_xy=0.2, cov_theta=0.1), timeout=remaining)
        try:
            while True:
                with self._condition:
                    remaining, _ = self._check(deadline, cancelled)
                try:
                    response = future.result(timeout=min(0.05, remaining))
                    break
                except grpc.FutureTimeoutError:
                    continue
            if not response.ok:
                raise RuntimeError(f'mapping rejected pose estimate: {response.detail}')
            with self._condition:
                self._feedback_after = self.node.get_clock().now().nanoseconds
                self._feedback_enabled = True
            authorize()
            with self._condition:
                self._check(deadline, cancelled)
            return self.node.get_clock().now().nanoseconds
        finally:
            if not future.done():
                future.cancel()

    def read_pose(self, stamp, deadline, cancelled, *, allow_before=False):
        """Wait for fresh map feedback at/just after a stationary scan's end.

        A maximum 100 ms acquisition skew is accepted only because the caller
        verifies stationary odometry across the entire capture/handoff interval.
        Messages must also remain fresh on both ROS and monotonic clocks.
        Tracking may opt into feedback before scan end only when it transports
        that pose through historical odometry to the observation's exact time.
        """
        if type(stamp) is not int or stamp <= 0:
            raise ValueError('positive observation time is required')
        with self._condition:
            while True:
                remaining, now = self._check(deadline, cancelled)
                earliest = stamp-100_000_000 if allow_before else stamp
                candidates = [pose for pose in self._poses
                    if (earliest <= pose['stamp_ns'] <= stamp+100_000_000 and
                        0 <= now-pose['stamp_ns'] <= self.max_age_s*1e9 and
                        0 <= time.monotonic()-pose['received'] <= self.max_age_s)]
                if candidates:
                    return copy.deepcopy(min(candidates, key=lambda pose: abs(pose['stamp_ns']-stamp)))
                self._condition.wait(timeout=min(0.05, remaining))

    def close(self):
        """Wake readers and release owned resources, preserving failed cleanup."""
        with self._condition:
            self._closed = True
            self._condition.notify_all()
        if self._subscription is not None:
            if not self.node.destroy_subscription(self._subscription):
                raise RuntimeError('mapping pose subscription cleanup failed')
            self._subscription = None
        if self._channel is not None:
            self._channel.close()
            self._channel = None
