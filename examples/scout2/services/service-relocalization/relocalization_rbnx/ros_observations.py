"""Bounded ROS input collector, attachable to an owned node and executor."""
from __future__ import annotations

from collections import deque
import copy
import threading
import time

from .observations import (ObservationUnavailable, check_stationary, prepare_pair,
                           stamp_ns, transform_matrix)


class ObservationCollector:
    """Collect independent scans from explicit Atlas-resolved ROS endpoints.

    The caller spins the node on another thread. Construction starts only
    subscriptions, never motion or matching. Each collector serves one capture
    attempt; invalid input latches a failure and requires a new collector.
    """

    def __init__(self, node, lidar_topic, odom_topic, *, base_frame, odom_frame,
                 expected_base_from_sensor, options):
        """Use best-effort volatile sensors, bounded history and historical TF."""
        from rclpy.qos import qos_profile_sensor_data
        from tf2_ros import Buffer, TransformListener
        from sensor_msgs.msg import PointCloud2
        from nav_msgs.msg import Odometry
        from rclpy.clock import JumpThreshold
        from rclpy.duration import Duration

        options.validate()
        if any(not isinstance(topic, str) or not topic.startswith('/') for topic in (lidar_topic, odom_topic)):
            raise ValueError('explicit absolute sensor endpoints are required')
        self.node, self.options = node, options
        self.base_frame, self.odom_frame = base_frame, odom_frame
        self.expected = copy.deepcopy(expected_base_from_sensor)
        self._lock = threading.Condition()
        self._clock_invalid = threading.Event()
        self._clouds, self._odom = deque(maxlen=4), deque(maxlen=24000)
        self._last_cloud_stamp = self._last_odom_stamp = self._last_clock = 0
        self._error, self._closed, self._capturing = None, False, False
        self._captured = None
        self._tracking_end = 0
        self._subscriptions, self._listener, self._jump_handle = [], None, None
        # No frame-graph service: the node is caller-owned and may be reused.
        self._buffer = Buffer()
        try:
            self._jump_handle = node.get_clock().create_jump_callback(
                JumpThreshold(min_forward=None, min_backward=Duration(nanoseconds=-1),
                              on_clock_change=True), post_callback=self._clock_jump)
            self._listener = TransformListener(self._buffer, node, spin_thread=False)
            self._subscriptions.append(node.create_subscription(PointCloud2, lidar_topic,
                self._on_cloud, qos_profile_sensor_data))
            self._subscriptions.append(node.create_subscription(Odometry, odom_topic,
                self._on_odom, qos_profile_sensor_data))
        except Exception:
            self.close()
            raise

    def _clock_jump(self, jump):
        """Invalidate even brief backward jumps or a ROS time-source switch."""
        # Do not take the collector lock from a clock callback: readers obtain
        # clock handles while holding that lock. Waiters check every 50 ms.
        self._clock_invalid.set()

    def _receive(self, message, cloud):
        """Latch malformed/reset streams; never swallow callback failures."""
        with self._lock:
            if self._closed or self._error is not None:
                return
            try:
                now = self.node.get_clock().now().nanoseconds
                if self._clock_invalid.is_set() or now <= 0 or now < self._last_clock:
                    raise ValueError('ROS clock is zero or moved backwards during capture')
                self._last_clock = now
                stamp = stamp_ns(message.header.stamp)
                if stamp > now or now-stamp > self.options.max_age_s*1e9:
                    raise ValueError('sensor timestamp is stale or future dated')
                previous = self._last_cloud_stamp if cloud else self._last_odom_stamp
                if stamp <= previous:
                    raise ValueError('sensor timestamp duplicated or moved backwards')
                # Bound memory before storing raw PointCloud2 data.
                if cloud and (len(message.data) > 64_000_000 or
                              message.width * message.height > 1_000_000):
                    raise ValueError('oversized PointCloud2 input')
                if cloud:
                    self._last_cloud_stamp = stamp
                    self._clouds.append((copy.deepcopy(message), time.monotonic()))
                else:
                    self._last_odom_stamp = stamp
                    self._odom.append(copy.deepcopy(message))
            except Exception as exc:
                self._error = exc
            self._lock.notify_all()

    def _on_cloud(self, message):
        self._receive(message, True)

    def _on_odom(self, message):
        self._receive(message, False)

    def _lookup(self, target, source, stamp):
        """Wait briefly for the exact ROS acquisition time, never use latest TF."""
        from rclpy.time import Time
        from rclpy.clock import ClockType
        from rclpy.duration import Duration
        return self._buffer.lookup_transform(target, source,
            Time(nanoseconds=stamp, clock_type=ClockType.ROS_TIME),
            timeout=Duration(seconds=0.1))

    def collect(self, timeout_s, cancelled):
        """Wait off the executor thread for fresh scans acquired after this call.

        Waits are at most 50 ms; timeouts use monotonic time, observations use
        ROS time. Expired buffered scans are replaced while waiting for odom
        brackets or TF; incoming timestamp failures still abort the request.
        All geometry, clock or callback failures abort without partial output.
        """
        import math
        if type(timeout_s) not in (int, float) or not math.isfinite(timeout_s) or not 0 < timeout_s <= 120:
            raise ValueError('capture timeout_s must be in (0, 120]')
        with self._lock:
            if self._capturing:
                raise RuntimeError('observation capture is already running')
            self._capturing = True
            self._captured = None
        try:
            return self._collect(timeout_s, cancelled)
        finally:
            with self._lock:
                self._capturing = False

    def _collect(self, timeout_s, cancelled):
        """Own one deadline and preserve explicit waiting diagnostics."""
        from tf2_ros import TransformException
        started = time.monotonic()
        deadline = started + timeout_s
        start_ros = self.node.get_clock().now().nanoseconds
        last_reason = 'waiting for two independent scans'
        while True:
            with self._lock:
                if self._closed:
                    raise RuntimeError('observation collector is closed')
                if self._error is not None:
                    raise RuntimeError(f'observation input failed: {self._error}') from self._error
                if cancelled.cancelled:
                    raise RuntimeError('observation capture cancelled')
                remaining = deadline-time.monotonic()
                if remaining <= 0:
                    raise TimeoutError(f'observation capture timed out: {last_reason}')
                now = self.node.get_clock().now().nanoseconds
                if self._clock_invalid.is_set() or now <= 0 or now < start_ros or now < self._last_clock:
                    raise ValueError('ROS clock is zero or moved backwards during capture')
                self._last_clock = now
                scans = []
                for message, received in self._clouds:
                    stamp = stamp_ns(message.header.stamp)
                    begin = stamp-self.options.scan_duration_ns if self.options.header_is_end else stamp
                    end = begin+self.options.scan_duration_ns
                    if begin > start_ros and received >= started:
                        if end > now:
                            last_reason = 'waiting for scan acquisition to complete'
                        elif now-end > self.options.max_age_s*1e9:
                            last_reason = 'waiting for fresh scans after buffered scans expired'
                        else:
                            scans.append(message)
                odometry = list(self._odom)
            if len(scans) >= 2:
                # Prefer recent independent acquisitions. Waiting for odom/TF
                # must not keep selecting an older pair until it expires.
                pair = next(((scans[i], scans[j]) for j in range(len(scans)-1, 0, -1)
                    for i in range(j-1, -1, -1)
                    if stamp_ns(scans[j].header.stamp)-stamp_ns(scans[i].header.stamp)
                    > self.options.scan_duration_ns), None)
                if pair is not None:
                    try:
                        result = prepare_pair(pair, odometry, self._lookup, base_frame=self.base_frame,
                            odom_frame=self.odom_frame, expected_base_from_sensor=self.expected,
                            now_ns=now, options=self.options)
                    except (ObservationUnavailable, TransformException) as exc:
                        last_reason = str(exc)
                    else:
                        # Recheck terminal conditions after decoding and TF work.
                        with self._lock:
                            final_ros = self.node.get_clock().now().nanoseconds
                            if self._closed or self._error is not None or cancelled.cancelled:
                                raise RuntimeError('observation capture invalidated before completion')
                            if time.monotonic() >= deadline:
                                raise TimeoutError('observation capture deadline exceeded during preparation')
                            if (self._clock_invalid.is_set() or final_ros < now or any(
                                    final_ros-item.end_stamp_ns > self.options.max_age_s*1e9
                                    for item in result)):
                                raise ValueError('observation expired or ROS clock reset during preparation')
                            self._captured = (result, pair)
                        return result
            with self._lock:
                self._lock.wait(timeout=min(0.05, max(0, deadline-time.monotonic())))

    def validate_current(self, timeout_s, cancelled):
        """Revalidate the last captured pair immediately before pose handoff.

        Return the initial stationary guard so post-handoff checks cover the
        intervening RPC and new acquisition as well. Caller serializes captures.
        """
        with self._lock:
            if self._captured is None:
                raise RuntimeError('no captured observations to validate for handoff')
            observations = self._captured[0]
        self.validate_after_matching(observations, timeout_s, cancelled)
        return min(observations[0].start_stamp_ns, observations[1].end_stamp_ns-
                   int(self.options.stationary_window_s*1e9))

    def collect_tracking(self, timeout_s, cancelled):
        """Capture one new scan during navigation with bounded distortion.

        No fabricated per-point times or deskew are used. Dense odometry must
        bound every point's possible displacement from scan-end base by 3 cm
        and 0.02 rad. The caller scores that fixed pose conservatively. Faster
        motion or missing odometry invalidates localization instead of allowing
        uncontrolled scan distortion to masquerade as geometry evidence.
        """
        import numpy as np
        from .observations import (Observation, _odom_geometry, cloud_xyz)
        from tf2_ros import TransformException
        started, deadline = time.monotonic(), time.monotonic()+timeout_s
        reason = 'waiting for a new tracking scan and odometry'
        while True:
            with self._lock:
                if self._closed or cancelled.cancelled or self._error is not None:
                    raise RuntimeError(f'tracking input closed/cancelled/invalid: {self._error}')
                if time.monotonic() >= deadline:
                    raise TimeoutError(reason)
                now = self.node.get_clock().now().nanoseconds
                if self._clock_invalid.is_set() or now <= 0 or now < self._last_clock:
                    raise ValueError('ROS clock changed during tracking')
                self._last_clock = now
                scans = [m for m, received in self._clouds if received > started]
                odom = list(self._odom)
            if scans:
                message = scans[-1]
                stamp = stamp_ns(message.header.stamp)
                begin = stamp-self.options.scan_duration_ns if self.options.header_is_end else stamp
                end = begin+self.options.scan_duration_ns
                if end > now:
                    reason = 'tracking scan acquisition has not completed'
                else:
                    try:
                        stamps = [stamp_ns(m.header.stamp) for m in odom]
                        guard_begin = min(begin, self._tracking_end) if self._tracking_end else begin
                        before = [i for i, t in enumerate(stamps) if t <= guard_begin]
                        after = [i for i, t in enumerate(stamps) if t >= end]
                        if not before or not after:
                            raise ObservationUnavailable('tracking odometry does not bracket scan')
                        samples = odom[before[-1]:after[0]+1]
                        times = stamps[before[-1]:after[0]+1]
                        if any(b-a > self.options.max_odom_gap_s*1e9 for a, b in zip(times, times[1:])):
                            raise ValueError('tracking odometry has a gap')
                        positions = [_odom_geometry(m, self.base_frame, self.odom_frame) for m in samples]
                        for a, b, ta, tb in zip(positions, positions[1:], times, times[1:]):
                            dt = (tb-ta)/1e9
                            turn = np.arccos(np.clip((np.trace(a[1].T @ b[1])-1)/2, -1, 1))
                            if (dt <= 0 or np.linalg.norm(a[0]-b[0]) > max(a[2], b[2])*dt+0.005 or
                                    turn > max(a[3], b[3])*dt+0.005):
                                raise ValueError('tracking odometry reset or discontinuity')
                        positions = [p for p, t in zip(positions, times) if t >= begin]
                        reference, attitude = positions[-1][:2]
                        for p, r, linear, angular in positions:
                            angle = np.arccos(np.clip((np.trace(attitude.T @ r)-1)/2, -1, 1))
                            if (np.linalg.norm(p-reference) > 0.03 or angle > 0.02 or
                                    linear*(end-begin)/1e9 > 0.03 or
                                    angular*(end-begin)/1e9 > 0.02):
                                raise ValueError('tracking scan distortion exceeds supported motion bounds')
                        source = message.header.frame_id
                        for t in (begin, end):
                            matrix = transform_matrix(self._lookup(self.base_frame, source, t), self.base_frame, source)
                            if not np.allclose(matrix, self.expected, rtol=0, atol=1e-4):
                                raise ValueError('tracking sensor calibration changed')
                        points = cloud_xyz(message)
                        ranges = np.linalg.norm(points, axis=1)
                        points = points[(ranges >= self.options.min_range_m) & (ranges <= self.options.max_range_m)]
                        points = points @ self.expected[:3, :3].T + self.expected[:3, 3]
                        _, first = np.unique(np.floor(points/self.options.voxel_size_m), axis=0, return_index=True)
                        points = np.ascontiguousarray(points[np.sort(first)])
                        if not 20 <= len(points) <= 10000:
                            raise ValueError('tracking scan has insufficient or excessive voxel points')
                        if self.node.get_clock().now().nanoseconds-end > self.options.max_age_s*1e9:
                            raise ValueError('tracking scan expired during decoding')
                        self._tracking_end = end
                        observation = Observation(points, tuple(self.expected[:3, 3]), begin, end)
                        self.validate_tracking(observation)
                        return observation
                    except (ObservationUnavailable, TransformException) as exc:
                        reason = str(exc)
            with self._lock:
                self._lock.wait(timeout=min(0.02, max(0, deadline-time.monotonic())))

    def validate_tracking(self, observation):
        """Recheck asynchronous input failures and clock freshness after computation."""
        with self._lock:
            now = self.node.get_clock().now().nanoseconds
            if self._closed or self._error is not None or self._clock_invalid.is_set():
                raise RuntimeError(f'tracking inputs invalidated: {self._error}')
            if (now < self._last_clock or not 0 <= now-observation.end_stamp_ns <= self.options.max_age_s*1e9):
                raise ValueError('tracking observation expired or ROS clock reset')
            self._last_clock = now

    def pose_at(self, observation, mapping_pose):
        """Transport fresh mapping feedback to the scan-end pose using odometry.

        The feedback may precede or follow scan end by up to 100 ms. Historical odom TF
        removes that skew during navigation; no latest transform or fabricated
        timestamp is substituted. Missing TF aborts the verification.
        """
        import math
        import numpy as np
        from .observations import rotation
        def matrix(stamp):
            """Read a rigid odom-to-base transform without sensor translation limits."""
            value = self._lookup(self.odom_frame, self.base_frame, stamp).transform
            result = np.eye(4)
            result[:3, :3] = rotation(value.rotation)
            result[:3, 3] = [value.translation.x, value.translation.y, value.translation.z]
            if not np.isfinite(result).all():
                raise ValueError('nonfinite historical tracking odometry TF')
            return result
        r, p, y = mapping_pose['roll'], mapping_pose['pitch'], mapping_pose['yaw']
        cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
        mapped = np.eye(4)
        mapped[:3, :3] = [[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
                         [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr], [-sp, cp*sr, cp*cr]]
        mapped[:3, 3] = [mapping_pose['x'], mapping_pose['y'], mapping_pose['z']]
        at_scan = mapped @ np.linalg.inv(matrix(mapping_pose['stamp_ns'])) @ matrix(observation.end_stamp_ns)
        if (np.linalg.norm(at_scan[:3, 3]-mapped[:3, 3]) > 0.05 or
                np.arccos(np.clip((np.trace(mapped[:3, :3].T @ at_scan[:3, :3])-1)/2, -1, 1)) > 0.05):
            raise ValueError('odometry pose-feedback skew exceeds supported bounds')
        return dict(mapping_pose, x=float(at_scan[0, 3]), y=float(at_scan[1, 3]),
                    z=float(at_scan[2, 3]), yaw=math.atan2(at_scan[1, 0], at_scan[0, 0]))

    def validate_after_matching(self, observations, timeout_s, cancelled, *, stationary_since_ns=None):
        """Recheck a captured pair through the ROS time at computation completion.

        This must run off the executor thread. Dense odometry must bracket the
        fixed completion horizon; the entire interval including computation is
        checked for motion. Historical and completion-time sensor TF must still
        agree with calibration. No result grants pose handoff or navigation.
        """
        import math
        if type(timeout_s) not in (int, float) or not math.isfinite(timeout_s) or not 0 < timeout_s <= 120:
            raise ValueError('validation timeout_s must be in (0, 120]')
        if stationary_since_ns is not None and (type(stationary_since_ns) is not int or
                not 0 < stationary_since_ns <= observations[0].start_stamp_ns):
            raise ValueError('stationary guard must precede the current observations')
        with self._lock:
            if self._capturing:
                raise RuntimeError('observation capture is already running')
            if self._captured is None or observations is not self._captured[0]:
                raise ValueError('observations do not belong to the current capture')
            self._capturing = True
            pair = self._captured[1]
        try:
            self._validate_after_matching(observations, pair, timeout_s, cancelled, stationary_since_ns)
        finally:
            with self._lock:
                self._capturing = False

    def _validate_after_matching(self, observations, pair, timeout_s, cancelled, stationary_since_ns=None):
        """Validate an older search window through uninterrupted stationary input.

        Global search can exceed one sensor freshness interval. Old scans remain
        usable only with dense stationary odometry through now and a continuously
        fresh live cloud stream; handoff still requires independent new scans.
        """
        import numpy as np
        from tf2_ros import TransformException
        deadline = time.monotonic()+timeout_s
        horizon = self.node.get_clock().now().nanoseconds
        guard = min(observations[0].start_stamp_ns, observations[1].end_stamp_ns-
                    int(self.options.stationary_window_s*1e9))
        if stationary_since_ns is not None:
            guard = min(guard, stationary_since_ns)
        last_reason = 'waiting for completion-time odometry and TF'
        while True:
            with self._lock:
                if self._closed or cancelled.cancelled:
                    raise RuntimeError('observation validation closed or cancelled')
                if self._error is not None:
                    raise RuntimeError(f'observation input failed: {self._error}') from self._error
                if time.monotonic() >= deadline:
                    raise TimeoutError(f'observation validation timed out: {last_reason}')
                now = self.node.get_clock().now().nanoseconds
                if self._clock_invalid.is_set() or now <= 0 or now < horizon or now < self._last_clock:
                    raise ValueError('ROS clock changed during observation validation')
                self._last_clock = now
                odometry = list(self._odom)
                if not self._clouds:
                    raise ValueError('no live lidar stream during computation validation')
                completed = [(stamp_ns(m.header.stamp)+(0 if self.options.header_is_end else
                    self.options.scan_duration_ns), received) for m, received in self._clouds]
                completed = [(end, received) for end, received in completed if end <= now]
                if not completed:
                    raise ObservationUnavailable('waiting for completed live lidar scan')
                latest_end, received = completed[-1]
                if (now-latest_end > self.options.max_age_s*1e9 or
                        time.monotonic()-received > self.options.max_age_s):
                    raise ValueError('live lidar stream became stale during computation')
            try:
                prepare_pair(pair, odometry, self._lookup, base_frame=self.base_frame,
                    odom_frame=self.odom_frame, expected_base_from_sensor=self.expected,
                    now_ns=observations[-1].end_stamp_ns, options=self.options)
                check_stationary(odometry, guard, horizon, self.base_frame, self.odom_frame, self.options)
                current = transform_matrix(self._lookup(self.base_frame, pair[0].header.frame_id, horizon),
                                           self.base_frame, pair[0].header.frame_id)
                if not np.allclose(current, self.expected, rtol=0, atol=1e-4):
                    raise ValueError('completion-time sensor TF differs from map calibration')
            except (ObservationUnavailable, TransformException) as exc:
                last_reason = str(exc)
            else:
                with self._lock:
                    final_ros = self.node.get_clock().now().nanoseconds
                    if self._closed or cancelled.cancelled or self._error is not None:
                        raise RuntimeError('observation validation invalidated before completion')
                    if time.monotonic() >= deadline:
                        raise TimeoutError('observation validation deadline exceeded')
                    if (self._clock_invalid.is_set() or final_ros < now or
                            final_ros-latest_end > self.options.max_age_s*1e9):
                        raise ValueError('live lidar expired or ROS clock reset during validation')
                return
            with self._lock:
                self._lock.wait(timeout=min(0.05, max(0, deadline-time.monotonic())))

    def close(self):
        """Wake waiters and destroy only owned subscriptions and TF listener."""
        with self._lock:
            self._closed = True
            self._lock.notify_all()
        errors, remaining = [], []
        for subscription in self._subscriptions:
            try:
                self.node.destroy_subscription(subscription)
            except Exception as exc:
                errors.append(exc)
                remaining.append(subscription)
        self._subscriptions = remaining
        if self._listener is not None:
            try:
                self._listener.unregister()
            except Exception as exc:
                errors.append(exc)
            else:
                self._listener = None
        if self._jump_handle is not None:
            try:
                self._jump_handle.unregister()
            except Exception as exc:
                errors.append(exc)
            else:
                self._jump_handle = None
        if errors:
            raise RuntimeError(f'observation cleanup failed: {errors}') from errors[0]
