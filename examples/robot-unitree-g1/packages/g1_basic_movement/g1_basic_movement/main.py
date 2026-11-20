#!/usr/bin/env python3
"""g1_basic_movement — Velocity-based basic movement skill for Unitree G1.

Exposes two MCP tools:
  - move: drive with linear/angular velocity for a specified duration
  - stop: immediately zero all velocities

Publishes geometry_msgs/TwistStamped on the configured twist topic
(default /cmd_vel) at 10 Hz to satisfy the chassis 300 ms watchdog.
"""
from __future__ import annotations

import logging
import math
import os
import threading
import time

from g1_basic_movement_mcp import (
    Move_Request,
    Move_Response,
    Stop_Request,
    Stop_Response,
)
from robonix_api import Err, Ok, Skill

logging.basicConfig(
    level=os.environ.get("G1_BASIC_MOVEMENT_LOG_LEVEL", "INFO").upper(),
    format="[g1_basic_movement] %(levelname)s %(message)s",
)
log = logging.getLogger("g1_basic_movement")

skill = Skill(id="g1_basic_movement", namespace="robonix/skill/basic_movement")

# ── Limits (match g1_chassis daemon clamps) ──────────────────────
_MAX_VX = 1.0   # m/s
_MAX_VY = 0.5   # m/s
_MAX_WZ = 2.0   # rad/s
_MAX_DURATION = 10.0  # seconds
_MIN_DURATION = 0.1
_PUBLISH_HZ = 10.0

# ── ROS2 state (initialized in on_activate) ──────────────────────
_node = None
_pub = None
_ros_thread = None
_stop_event = threading.Event()
_move_lock = threading.Lock()
_moving = False


def _clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


def _make_twist(vx: float, vy: float, wz: float):
    """Build a TwistStamped message."""
    import builtin_interfaces.msg
    import geometry_msgs.msg

    msg = geometry_msgs.msg.TwistStamped()
    msg.header.stamp = _node.get_clock().now().to_msg()
    msg.twist.linear.x = float(vx)
    msg.twist.linear.y = float(vy)
    msg.twist.linear.z = 0.0
    msg.twist.angular.x = 0.0
    msg.twist.angular.y = 0.0
    msg.twist.angular.z = float(wz)
    return msg


def _publish(vx: float, vy: float, wz: float) -> None:
    if _pub is not None and _node is not None:
        _pub.publish(_make_twist(vx, vy, wz))


def _publish_zero() -> None:
    _publish(0.0, 0.0, 0.0)


def _ros_spin() -> None:
    """Background thread: spin the ROS2 node."""
    import rclpy

    try:
        rclpy.spin(_node)
    except Exception:
        pass


# ── MCP tools ────────────────────────────────────────────────────

@skill.mcp("robonix/skill/basic_movement/move")
def move(req: Move_Request) -> Move_Response:
    """Move the robot with specified velocity for a given duration."""
    global _moving

    vx = _clamp(float(req.vx), -_MAX_VX, _MAX_VX)
    vy = _clamp(float(req.vy), -_MAX_VY, _MAX_VY)
    wz = _clamp(float(req.wz), -_MAX_WZ, _MAX_WZ)
    duration_s = _clamp(float(req.duration_s), _MIN_DURATION, _MAX_DURATION)

    if _node is None:
        return Move_Response(accepted=False, detail="ROS2 node not initialized")

    with _move_lock:
        if _moving:
            return Move_Response(
                accepted=False,
                detail="a movement is already in progress; call stop first",
            )
        _moving = True
        _stop_event.clear()

    log.info("move vx=%.3f vy=%.3f wz=%.3f duration=%.2fs", vx, vy, wz, duration_s)

    try:
        deadline = time.monotonic() + duration_s
        period = 1.0 / _PUBLISH_HZ
        while time.monotonic() < deadline:
            if _stop_event.is_set():
                break
            _publish(vx, vy, wz)
            time.sleep(period)
    finally:
        _publish_zero()
        with _move_lock:
            _moving = False

    return Move_Response(accepted=True, detail="movement completed")


@skill.mcp("robonix/skill/basic_movement/stop")
def stop(req: Stop_Request) -> Stop_Response:
    """Emergency stop — immediately zero all velocities."""
    _stop_event.set()
    _publish_zero()
    log.info("stop: velocities zeroed")
    return Stop_Response(accepted=True, detail="velocities zeroed")


# ── Lifecycle ────────────────────────────────────────────────────

@skill.on_init
def initialize(config: dict):
    """Read config; ROS2 is started on activate."""
    return Ok()


@skill.on_activate
def activate():
    """Start the ROS2 node and publisher."""
    global _node, _pub, _ros_thread

    import rclpy
    from rclpy.node import Node

    twist_topic = "/cmd_vel"

    if not rclpy.ok():
        rclpy.init(args=[])

    _node = Node("g1_basic_movement_skill")
    _pub = _node.create_publisher(
        __import__("geometry_msgs.msg", fromlist=["TwistStamped"]).TwistStamped,
        twist_topic,
        10,
    )

    _ros_thread = threading.Thread(target=_ros_spin, daemon=True)
    _ros_thread.start()

    log.info("ROS2 node started, publishing TwistStamped on %s", twist_topic)
    return Ok()


@skill.on_deactivate
def deactivate():
    """Stop movement and shut down the ROS2 node."""
    global _node, _pub, _ros_thread

    _stop_event.set()
    _publish_zero()

    if _node is not None:
        _node.destroy_node()
        _node = None
        _pub = None

    import rclpy
    if rclpy.ok():
        rclpy.shutdown()

    if _ros_thread is not None:
        _ros_thread.join(timeout=2.0)
        _ros_thread = None

    log.info("ROS2 node stopped")
    return Ok()


if __name__ == "__main__":
    skill.run()
