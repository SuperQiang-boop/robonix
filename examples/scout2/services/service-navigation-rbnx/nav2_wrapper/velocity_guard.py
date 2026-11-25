"""Final velocity guard and passive per-NavigateToPose trajectory recorder."""

from __future__ import annotations

import json
import copy
import math
import os
import threading
import sys
import time
from collections import deque
from pathlib import Path

import rclpy
from action_msgs.msg import GoalStatus, GoalStatusArray
from action_msgs.srv import CancelGoal
from geometry_msgs.msg import Twist
from nav2_msgs.action import NavigateToPose
from nav2_msgs.msg import SpeedLimit
from nav_msgs.msg import OccupancyGrid, Odometry, Path as NavPath
from sensor_msgs.msg import LaserScan
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener

from nav2_wrapper.configuration import resolve_velocity_output_topic
from nav2_wrapper.rotation_guard_core import GuardLimits, RotationGuard, normalize_uuid_octets
from nav2_wrapper.velocity_limit import bounded_linear_velocity


def _yaw(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def _uuid_text(raw) -> str:
    return bytes(raw).hex()


class VelocityGuardNode(Node):
    def __init__(self):
        # Validate before constructing the ROS node or creating any endpoint.
        # An invalid/empty/relative output must fail closed with no publisher.
        output_topic = resolve_velocity_output_topic({})
        max_linear_speed_mps = float(
            os.environ["ROBONIX_NAV_MAX_LINEAR_SPEED_MPS"]
        )
        default_percentage = float(
            os.environ["ROBONIX_NAV_DEFAULT_SPEED_PERCENTAGE"]
        )
        speed_limit_topic = os.environ[
            "ROBONIX_NAV_GUARD_SPEED_LIMIT_TOPIC"
        ]
        if (
            not math.isfinite(max_linear_speed_mps)
            or max_linear_speed_mps <= 0.0
        ):
            raise ValueError(
                "ROBONIX_NAV_MAX_LINEAR_SPEED_MPS must be finite and positive"
            )
        if not 0.0 < default_percentage <= 100.0:
            raise ValueError(
                "ROBONIX_NAV_DEFAULT_SPEED_PERCENTAGE must be in (0, 100]"
            )
        super().__init__("robonix_velocity_guard")
        self.set_parameters([Parameter('use_sim_time', value=(
            os.environ.get('ROBONIX_NAV_GUARD_USE_SIM_TIME', 'false') == 'true'))])
        self._localization_dir = os.environ.get('RELOCALIZATION_COORDINATION_DIR')
        self._localization_check = None
        self._localization_angular = None
        self._last_localization_cancel = 0.0
        self._stationary = deque(maxlen=100)
        self._mapping_identity = None
        self._checked_identity = None
        self._map_ready = False
        self._odom_received_at = None
        self._odom_stamp_ns = 0
        self._scan_received_at = None
        self._scan_stamp_ns = 0
        self._scan_frame = ''
        self._costmap_received_at = None
        self._tf_buffer = None
        self._tf_listener = None
        if self._localization_dir:
            sys.path.insert(0, os.environ['RELOCALIZATION_PACKAGE_ROOT'])
            from relocalization_rbnx.navigation import check
            provider = os.environ.get('RELOCALIZATION_PROVIDER_ID', 'relocalization')
            self._localization_check = lambda: check(self._localization_dir, provider)
            self._localization_angular = float(os.environ['ROBONIX_RELOCALIZATION_MAX_ANGULAR_RPS'])
            if not math.isfinite(self._localization_angular) or not 0 < self._localization_angular <= 0.2:
                raise ValueError('invalid relocalization angular speed bound')
        limits = GuardLimits(
            terminal_xy_m=float(os.getenv("ROBONIX_GUARD_TERMINAL_XY_M", "0.45")),
            terminal_timeout_s=float(os.getenv("ROBONIX_GUARD_TERMINAL_TIMEOUT_S", "15.0")),
            terminal_no_progress_s=float(os.getenv("ROBONIX_GUARD_NO_PROGRESS_S", "3.0")),
            global_spin_timeout_s=float(os.getenv("ROBONIX_GUARD_GLOBAL_TIMEOUT_S", "25.0")),
            global_spin_limit_rad=float(os.getenv("ROBONIX_GUARD_GLOBAL_ROTATION_RAD", "6.783185307")),
        )
        self.guard = RotationGuard(limits)
        self._lock = threading.Lock()
        self._active_uuid = ""
        self._active_uuid_raw = None
        self._robot_pose = None
        self._goal_pose = None
        self._last_cmd = (0.0, 0.0)
        self._cancel_sent = False
        self._trace = None
        self._trace_dir = Path(os.getenv("ROBONIX_NAV_TRACE_DIR", "/tmp/robonix-nav-traces"))
        self._trace_dir.mkdir(parents=True, exist_ok=True)
        self._scan_log = (self._trace_dir / "scan-anomalies.jsonl").open("a", encoding="utf-8")
        self._front_min_history = deque(maxlen=20)
        self._last_scan_summary_at = 0.0
        self._max_linear_speed_mps = max_linear_speed_mps
        self._speed_percentage = default_percentage
        self._effective_linear_speed_mps = (
            max_linear_speed_mps * default_percentage / 100.0
        )
        if self._localization_dir:
            self._tf_buffer = Buffer()
            self._tf_listener = TransformListener(self._tf_buffer, self)
            map_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                 durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.create_subscription(OccupancyGrid,
                os.environ.get('ROBONIX_NAV_GUARD_MAP_TOPIC', '/map'),
                self._on_map, map_qos)
            self.create_subscription(OccupancyGrid, '/global_costmap/costmap',
                self._on_costmap, map_qos)

        self._pub = self.create_publisher(Twist, output_topic, 10)
        self.create_subscription(Twist, "/cmd_vel_guard_input", self._on_cmd, 20)
        self.create_subscription(
            SpeedLimit, speed_limit_topic, self._on_speed_limit, 10
        )
        self.create_subscription(Odometry, os.environ.get('ROBONIX_NAV_GUARD_ODOM_TOPIC', '/odom'), self._on_odom, qos_profile_sensor_data)
        self.create_subscription(NavPath, "/plan", self._on_plan, 10)
        self.create_subscription(LaserScan,
            os.environ.get('ROBONIX_NAV_GUARD_SCAN_TOPIC', '/scanner/scan'),
            self._on_scan, qos_profile_sensor_data)
        self.create_subscription(
            GoalStatusArray, "/navigate_to_pose/_action/status", self._on_status, 10
        )
        self.create_subscription(
            NavigateToPose.Impl.FeedbackMessage,
            "/navigate_to_pose/_action/feedback",
            self._on_feedback,
            20,
        )
        self._cancel = self.create_client(CancelGoal, "/navigate_to_pose/_action/cancel_goal")
        self.create_timer(0.05, self._publish_latched_zero)
        self.get_logger().info(
            f"guard active; output={output_topic}; "
            f"max_linear={max_linear_speed_mps:g}m/s; "
            f"default={default_percentage:g}%; traces={self._trace_dir}"
        )

    def _on_speed_limit(self, msg: SpeedLimit) -> None:
        """Track the live Nav2 limit and retain the configured hard maximum."""
        value = float(msg.speed_limit)
        if not math.isfinite(value) or value < 0.0:
            self.get_logger().error(f"ignoring invalid speed limit: {value}")
            return
        with self._lock:
            if value == 0.0:
                percentage = 100.0
                linear_limit_mps = self._max_linear_speed_mps
            elif msg.percentage:
                percentage = min(100.0, max(0.0, value))
                linear_limit_mps = (
                    self._max_linear_speed_mps * percentage / 100.0
                )
            else:
                linear_limit_mps = min(self._max_linear_speed_mps, value)
                percentage = (
                    linear_limit_mps / self._max_linear_speed_mps * 100.0
                )
            self._speed_percentage = percentage
            self._effective_linear_speed_mps = linear_limit_mps
            self._event(
                "speed_limit",
                percentage=percentage,
                linear_limit_mps=linear_limit_mps,
                source_percentage=bool(msg.percentage),
            )

    def _event(self, kind: str, **values) -> None:
        if self._trace is None:
            return
        record = {"t_wall": time.time(), "event": kind, **values}
        self._trace.write(json.dumps(record, separators=(",", ":")) + "\n")
        self._trace.flush()

    def _begin_goal(self, goal_uuid: str, raw_uuid) -> None:
        if goal_uuid == self._active_uuid:
            return
        if self._trace is not None:
            self._event("trace_closed", reason="new_goal")
            self._trace.close()
        self._active_uuid = goal_uuid
        self._active_uuid_raw = normalize_uuid_octets(raw_uuid)
        self._cancel_sent = False
        self._robot_pose = None
        self._goal_pose = None
        self.guard.begin_goal(goal_uuid)
        stamp = time.strftime("%Y%m%d-%H%M%S")
        self._trace = (self._trace_dir / f"{stamp}-{goal_uuid}.jsonl").open("a", encoding="utf-8")
        self._event("goal_active", uuid=goal_uuid)

    def _on_status(self, msg: GoalStatusArray) -> None:
        with self._lock:
            active = [
                item for item in msg.status_list
                if item.status in (GoalStatus.STATUS_ACCEPTED, GoalStatus.STATUS_EXECUTING,
                                   GoalStatus.STATUS_CANCELING)
            ]
            if active:
                item = active[-1]
                self._begin_goal(_uuid_text(item.goal_info.goal_id.uuid), item.goal_info.goal_id.uuid)
            self._event(
                "status",
                values=[{"uuid": _uuid_text(s.goal_info.goal_id.uuid), "status": int(s.status)}
                        for s in msg.status_list],
            )
            if not active and self._active_uuid:
                self._event("goal_inactive")
                self.guard.clear_if_idle()

    def _on_feedback(self, msg) -> None:
        with self._lock:
            uuid = _uuid_text(msg.goal_id.uuid)
            if uuid and uuid != self._active_uuid:
                self._begin_goal(uuid, msg.goal_id.uuid)
            pose = msg.feedback.current_pose.pose
            self._robot_pose = (pose.position.x, pose.position.y, _yaw(pose.orientation))
            self._event(
                "feedback", pose=self._robot_pose,
                distance_remaining=float(msg.feedback.distance_remaining),
                recoveries=int(msg.feedback.number_of_recoveries),
            )

    def _on_plan(self, msg: NavPath) -> None:
        if not msg.poses:
            return
        with self._lock:
            pose = msg.poses[-1].pose
            self._goal_pose = (pose.position.x, pose.position.y, _yaw(pose.orientation))
            self._event("plan", frame=msg.header.frame_id, points=len(msg.poses), goal=self._goal_pose)

    def _on_odom(self, msg: Odometry) -> None:
        """Observe motion and retain fresh measurements for transition stop acknowledgement."""
        with self._lock:
            pose = msg.pose.pose
            yaw = _yaw(pose.orientation)
            if math.isfinite(yaw):
                self.guard.observe_odom(yaw)
            if self._localization_dir:
                stamp = msg.header.stamp.sec*1_000_000_000+msg.header.stamp.nanosec
                age = self.get_clock().now().nanoseconds-stamp
                twist = msg.twist.twist
                values = (pose.position.x, pose.position.y, pose.position.z, yaw,
                    twist.linear.x, twist.linear.y, twist.linear.z,
                    twist.angular.x, twist.angular.y, twist.angular.z)
                valid = (bool(msg.header.frame_id) and bool(msg.child_frame_id)
                         and all(math.isfinite(x) for x in values)
                         and all(math.isfinite(x) for x in (
                             pose.orientation.x, pose.orientation.y,
                             pose.orientation.z, pose.orientation.w))
                         and sum(x*x for x in (pose.orientation.x, pose.orientation.y,
                             pose.orientation.z, pose.orientation.w)) > 1e-6
                         and 0 <= age <= 500_000_000)
                self._odom_received_at = time.monotonic() if valid else None
                self._odom_stamp_ns = stamp
                if (not 0 <= age <= 200_000_000 or not all(math.isfinite(x) for x in values) or
                        math.sqrt(sum(x*x for x in values[4:7])) > 0.02 or
                        math.sqrt(sum(x*x for x in values[7:])) > 0.02):
                    self._stationary.clear()
                else:
                    self._stationary.append((time.monotonic(), stamp, values[:4]))
            self._event(
                "odom", pose=(pose.position.x, pose.position.y, yaw),
                twist=(msg.twist.twist.linear.x, msg.twist.twist.angular.z),
            )

    def _on_scan(self, msg: LaserScan) -> None:
        """Persist sudden/isolated close returns even when no goal is active."""
        stamp = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        age = self.get_clock().now().nanoseconds - stamp
        valid = (bool(msg.header.frame_id) and len(msg.ranges) > 0
                 and math.isfinite(msg.range_min) and math.isfinite(msg.range_max)
                 and 0 <= msg.range_min < msg.range_max
                 and math.isfinite(msg.angle_min) and math.isfinite(msg.angle_increment)
                 and msg.angle_increment != 0 and 0 <= age <= 500_000_000
                 and any(value == math.inf or (math.isfinite(value) and
                         msg.range_min <= value <= msg.range_max) for value in msg.ranges))
        with self._lock:
            self._scan_received_at = time.monotonic() if valid else None
            self._scan_stamp_ns = stamp
            self._scan_frame = msg.header.frame_id
        front = []
        for index, value in enumerate(msg.ranges):
            angle = msg.angle_min + index * msg.angle_increment
            angle = math.atan2(math.sin(angle), math.cos(angle))
            if (
                abs(angle) <= math.radians(60.0)
                and math.isfinite(value)
                and msg.range_min <= value <= msg.range_max
            ):
                front.append((index, angle, float(value)))
        if not front:
            return
        ordered = sorted(value for _, _, value in front)
        front_min = ordered[0]
        front_p05 = ordered[min(len(ordered) - 1, int(len(ordered) * 0.05))]
        baseline = None
        if self._front_min_history:
            history = sorted(self._front_min_history)
            baseline = history[len(history) // 2]
        sudden = baseline is not None and baseline - front_min >= 0.40 and front_min <= 2.0

        isolated = []
        by_index = {index: value for index, _, value in front}
        for index, angle, value in front:
            if value > 2.0:
                continue
            neighbors = [by_index.get(index + offset) for offset in (-2, -1, 1, 2)]
            close_neighbors = sum(
                other is not None and abs(other - value) <= 0.20 for other in neighbors
            )
            if close_neighbors <= 1:
                isolated.append((round(angle, 5), round(value, 4)))
        now = time.monotonic()
        periodic = now - self._last_scan_summary_at >= 1.0
        if sudden or periodic:
            record = {
                "t_wall": time.time(),
                "stamp": msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9,
                "frame": msg.header.frame_id,
                "front_min": round(front_min, 4),
                "front_p05": round(front_p05, 4),
                "baseline_min": None if baseline is None else round(baseline, 4),
                "sudden": sudden,
                "isolated": isolated[:40],
                "front_under_2m": [
                    (round(angle, 5), round(value, 4))
                    for _, angle, value in front if value <= 2.0
                ] if sudden else [],
                "range_min": float(msg.range_min),
                "range_max": float(msg.range_max),
            }
            self._scan_log.write(json.dumps(record, separators=(",", ":")) + "\n")
            self._scan_log.flush()
            self._last_scan_summary_at = now
        self._front_min_history.append(front_min)

    def _on_map(self, msg: OccupancyGrid) -> None:
        """Accept nonempty occupancy without expiring a stationary latched map."""
        from relocalization_rbnx.coordination import read_record
        info = msg.info
        with self._lock:
            try:
                state = read_record(Path(self._localization_dir) / 'mapping.json', 0.6)
                if state.get('ready') is not True:
                    self._map_ready = False
                    return
                self._mapping_identity = (state['process_id'], state['load_id'])
            except (OSError, ValueError, TypeError, KeyError, RuntimeError):
                self._map_ready = False
                return
            self._map_ready = (msg.header.frame_id == 'map'
                and info.width > 0 and info.height > 0
                and math.isfinite(info.resolution) and info.resolution > 0
                and len(msg.data) == info.width * info.height
                and any(value >= 0 for value in msg.data))

    def _on_costmap(self, msg: OccupancyGrid) -> None:
        """Track live Nav2 costmap updates independently of static map content."""
        info = msg.info
        valid = (msg.header.frame_id == 'map' and info.width > 0 and info.height > 0
                 and math.isfinite(info.resolution) and info.resolution > 0
                 and len(msg.data) == info.width * info.height)
        with self._lock:
            self._costmap_received_at = time.monotonic() if valid else None

    def _mapping_inputs_ready(self) -> bool:
        """Check this mapping load's map, live sensors and TF before forwarding.

        Called with the guard lock held. A changed process/load invalidates the
        previous occupancy sample; initial startup may use its latched sample.
        """
        from relocalization_rbnx.coordination import read_record
        try:
            mapping = read_record(Path(self._localization_dir) / 'mapping.json', 0.6)
            identity = (mapping['process_id'], mapping['load_id'])
            if self._mapping_identity is not None and identity != self._mapping_identity:
                self._map_ready = False
            self._mapping_identity = identity
            if mapping.get('ready') is not True or mapping.get('mode') != 'mapping':
                return False
            now = time.monotonic()
            stamp_now = self.get_clock().now().nanoseconds
            if not self._map_ready or self._tf_buffer is None:
                return False
            if (self._costmap_received_at is None or
                    not 0 <= now - self._costmap_received_at <= 1.0):
                return False
            for received, stamp in ((self._odom_received_at, self._odom_stamp_ns),
                                    (self._scan_received_at, self._scan_stamp_ns)):
                if received is None or not 0 <= now - received <= 0.5:
                    return False
                if not 0 <= stamp_now - stamp <= 500_000_000:
                    return False
            transform = self._tf_buffer.lookup_transform('map', 'base_link', Time())
            stamp = transform.header.stamp
            age = stamp_now - (stamp.sec * 1_000_000_000 + stamp.nanosec)
            if not -100_000_000 <= age <= 500_000_000:
                return False
            pose = transform.transform
            if not all(math.isfinite(value) for value in (
                    pose.translation.x, pose.translation.y, pose.translation.z,
                    pose.rotation.x, pose.rotation.y, pose.rotation.z, pose.rotation.w)):
                return False
            self._tf_buffer.lookup_transform('base_link', self._scan_frame, Time())
            return True
        except (OSError, KeyError, ValueError, TypeError, RuntimeError, TransformException):
            return False

    def _navigation_ready(self) -> bool:
        """Evaluate one mapping identity, failing if it changes during validation."""
        from relocalization_rbnx.coordination import read_record
        self._checked_identity = None
        try:
            path = Path(self._localization_dir) / 'mapping.json'
            before = read_record(path, 0.6)
            fields = ('process_id', 'load_id', 'mode')
            identity = tuple(before[key] for key in fields)
            if not self._localization_check()[0]:
                return False
            if before.get('mode') == 'mapping' and not self._mapping_inputs_ready():
                return False
            after = read_record(path, 0.6)
            if after.get('ready') is not True or identity != tuple(after[key] for key in fields):
                return False
            self._checked_identity = identity
            return True
        except (OSError, KeyError, ValueError, TypeError, RuntimeError):
            return False

    def _on_cmd(self, msg: Twist) -> None:
        """Gate Nav2 output on verified localization or healthy mapping inputs."""
        with self._lock:
            if self._localization_check is not None and not self._navigation_ready():
                self._pub.publish(Twist())
                self._cancel_for_localization()
                return
            if self._localization_angular is not None:
                msg = copy.deepcopy(msg)
                msg.angular.z = max(-self._localization_angular, min(self._localization_angular, msg.angular.z))
            x, y, limited = bounded_linear_velocity(
                float(msg.linear.x),
                float(msg.linear.y),
                self._effective_linear_speed_mps,
            )
            output = msg
            if limited:
                output = Twist()
                output.linear.x = x
                output.linear.y = y
                output.linear.z = msg.linear.z
                output.angular.x = msg.angular.x
                output.angular.y = msg.angular.y
                output.angular.z = msg.angular.z
            self._last_cmd = (math.hypot(x, y), float(output.angular.z))
            reason = self.guard.evaluate(
                self.get_clock().now().nanoseconds / 1e9,
                self._last_cmd[0], self._last_cmd[1], self._robot_pose, self._goal_pose,
            )
            self._event(
                "cmd", linear=self._last_cmd[0], angular=self._last_cmd[1],
                requested_linear=math.hypot(msg.linear.x, msg.linear.y),
                linear_limit_mps=self._effective_linear_speed_mps,
                linear_limited=limited,
                terminal_rotation=self.guard.terminal_rotation,
                continuous_rotation=self.guard.spin_rotation,
                latched=bool(reason),
            )
            if reason:
                self._trip(reason)
                self._pub.publish(Twist())
            else:
                self._pub.publish(output)

    def _trip(self, reason: str) -> None:
        if self._cancel_sent:
            return
        self._cancel_sent = True
        self.get_logger().error(f"ROTATION GUARD TRIPPED uuid={self._active_uuid}: {reason}")
        self._event("guard_trip", reason=reason)
        if self._active_uuid_raw is None or not self._cancel.service_is_ready():
            return
        request = CancelGoal.Request()
        request.goal_info.goal_id.uuid = self._active_uuid_raw
        self._cancel.call_async(request)
        self._event("cancel_requested", uuid=self._active_uuid)

    def _publish_latched_zero(self) -> None:
        """Stop invalid autonomous motion while leaving idle manual mapping free."""
        with self._lock:
            mapping = False
            if self._localization_check is not None:
                from relocalization_rbnx.navigation import mapping_active
                mapping = mapping_active(self._localization_dir)
                valid = self._navigation_ready()
                if not valid:
                    if not mapping or self._active_uuid or any(self._last_cmd):
                        self._pub.publish(Twist())
                        self._last_cmd = (0.0, 0.0)
                    self._cancel_for_localization()
                self._write_localization_stop(not valid)
            if self.guard.latched_reason and (not mapping or self._active_uuid):
                self._pub.publish(Twist())

    def _cancel_for_localization(self):
        """Repeatedly cancel all NavigateToPose actions while localization is invalid."""
        now = time.monotonic()
        if now-self._last_localization_cancel < 0.5 or not self._cancel.service_is_ready():
            return
        self._last_localization_cancel = now
        # Empty UUID and zero stamp are the ROS action protocol's cancel-all.
        self._cancel.call_async(CancelGoal.Request())

    def _write_localization_stop(self, blocked):
        """Acknowledge a map transition only after continuously stationary odometry."""
        from relocalization_rbnx.coordination import read_record, write_record
        try:
            directory = Path(self._localization_dir)
            mapping = read_record(directory / 'mapping.json', 0.6)
            now = time.monotonic()
            samples = list(self._stationary)
            stopped = blocked and bool(samples) and now-samples[-1][0] < 0.15
            stopped = stopped and samples[-1][0]-samples[0][0] >= 0.5
            if stopped:
                reference = samples[-1][2]
                stopped = all(math.dist(p[:3], reference[:3]) < 0.01 and
                    abs(math.remainder(p[3]-reference[3], 2*math.pi)) < 0.01
                    for _, _, p in samples)
                stopped = stopped and all(a[1] < b[1] and b[0]-a[0] <= 0.1
                    for a, b in zip(samples, samples[1:]))
            write_record(directory / 'navigation.json',
                dict(load_id=mapping['load_id'], stopped=bool(stopped),
                     mapping_process_id=mapping['process_id'],
                     ready=(not blocked and mapping.get('ready') is True and
                         self._checked_identity == (mapping['process_id'],
                             mapping['load_id'], mapping['mode']))))
        except (OSError, ValueError, KeyError, RuntimeError) as exc:
            self.get_logger().error(f'cannot acknowledge localization stop: {exc}')


def main() -> None:
    rclpy.init()
    node = VelocityGuardNode()
    try:
        rclpy.spin(node)
    except ExternalShutdownException:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
