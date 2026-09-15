#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""realsense_camera_rbnx — Intel RealSense D435i RGBD primitive
(capability_id=realsense_camera).

Owns `robonix/primitive/camera/*`. The D435i has an internal IMU but
we deliberately do NOT atlas-route it under `primitive/imu/imu`
(MID-360 IMU is canonical for the ranger). Subscribers needing the
camera IMU directly can read /<camera_name>/imu.

Capability surface:
  primitive/camera/driver         rpc gRPC (lifecycle)
  primitive/camera/rgb            topic_out ROS2 (continuous, raw)
  primitive/camera/depth          topic_out ROS2 (continuous, raw)
  primitive/camera/snapshot       rpc MCP (one-shot RGB JPEG — VLM-facing)
  primitive/camera/depth_snapshot rpc MCP (one-shot depth as 8-bit JPEG)
  primitive/camera/extrinsics     topic_out ROS2 (latched TransformStamped)

Lifecycle:
    on_init      — spawn rs_launch.py with camera_name + profiles → wait
                   for first RGB frame → subscribe rgb+depth → declare
                   rgb + depth topic_out + snapshot + depth_snapshot.
    on_shutdown  — kill realsense subprocess.

Config (from manifest):
    camera_name        default "camera_435i"
    rgb_topic          default "/<camera_name>/color/image_raw"
    depth_topic        default "/<camera_name>/aligned_depth_to_color/image_raw"
    rgb_profile        default "640x480x30"
    depth_profile      default "848x480x30"
    align_depth        default true
    enable_imu         default true   (published, NOT atlas-routed)
    enable_sync        default true
    sentinel_timeout_s default 30.0
"""
from __future__ import annotations

import logging
import os
import signal
import subprocess
import threading
import time
from io import BytesIO
from pathlib import Path

import numpy as np

from robonix_api import Primitive, Ok, Err

logging.basicConfig(
    level=os.environ.get("REALSENSE_LOG_LEVEL", "INFO"),
    format="[realsense] %(message)s",
)
log = logging.getLogger("realsense")

cap = Primitive(id="realsense_camera", namespace="robonix/primitive/camera")


def _pump_output(stream, tag: str) -> None:
    """Forward a child process's merged stdout/stderr into scribe via the
    package logger — one unified log stream, no side-car *.log file."""
    for raw in iter(stream.readline, b""):
        line = raw.decode(errors="replace").rstrip()
        if line:
            log.info("[%s] %s", tag, line)

_pkg_root: Path = Path(__file__).resolve().parent.parent
_rs_proc: subprocess.Popen | None = None

# ── snapshot state ───────────────────────────────────────────────────────────
_state_lock = threading.Lock()
_latest_rgb_jpeg: bytes | None = None
_latest_depth_jpeg: bytes | None = None
_rgb_frame_id: str = "camera_435i_color_optical_frame"
_extrinsics_pub = None
_depth_frame_id: str = "camera_435i_depth_optical_frame"


def _spawn_realsense(cfg: dict) -> None:
    """Launch ros2 launch realsense2_camera rs_launch.py with config args."""
    global _rs_proc
    cam = cfg.get("camera_name", "camera_435i")
    enable_imu = bool(cfg.get("enable_imu", True))
    args = [
        "ros2", "launch", "realsense2_camera", "rs_launch.py",
        "camera_namespace:=/",
        f"camera_name:={cam}",
        f"enable_gyro:={'true' if enable_imu else 'false'}",
        f"enable_accel:={'true' if enable_imu else 'false'}",
        f"align_depth.enable:={'true' if cfg.get('align_depth', True) else 'false'}",
        f"enable_sync:={'true' if cfg.get('enable_sync', True) else 'false'}",
        "publish_tf:=true",  # rtabmap consumes camera_link → optical_frame TFs
        f"spatial_filter.enable:={'true' if cfg.get('spatial_filter', True) else 'false'}",
        f"temporal_filter.enable:={'true' if cfg.get('temporal_filter', True) else 'false'}",
        f"hole_filling_filter.enable:={'true' if cfg.get('hole_filling_filter', False) else 'false'}",
        f"rgb_camera.color_profile:={cfg.get('rgb_profile', '640x480x30')}",
        f"depth_module.depth_profile:={cfg.get('depth_profile', '848x480x30')}",
    ]
    if enable_imu:
        args.append("unite_imu_method:=2")
    log.info("spawning realsense (cam=%s)", cam)
    log.debug("launch args: %s", " ".join(args))
    _rs_proc = subprocess.Popen(
        args, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
        start_new_session=True,
    )
    threading.Thread(target=_pump_output, args=(_rs_proc.stdout, "realsense"),
                     daemon=True).start()


def _kill_realsense() -> None:
    p = _rs_proc
    if p is None or p.poll() is not None:
        return
    try:
        os.killpg(os.getpgid(p.pid), signal.SIGTERM)
    except ProcessLookupError:
        return
    try:
        p.wait(timeout=5.0)
    except subprocess.TimeoutExpired:
        try:
            os.killpg(os.getpgid(p.pid), signal.SIGKILL)
        except ProcessLookupError:
            pass


# ── image conversion ─────────────────────────────────────────────────────────
def _ros_image_to_jpeg(msg) -> bytes:
    """Encode a sensor_msgs/Image into JPEG bytes.
    Supports: rgb8, bgr8, rgba8, bgra8, mono8, 16uc1, 32fc1."""
    h, w = msg.height, msg.width
    enc = msg.encoding.lower()
    if enc == "rgb8":
        arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(h, w, 3)
    elif enc == "bgr8":
        arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(h, w, 3)[:, :, ::-1]
    elif enc == "rgba8":
        arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(h, w, 4)[:, :, :3]
    elif enc == "bgra8":
        arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(h, w, 4)[:, :, :3][:, :, ::-1]
    elif enc == "mono8":
        arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(h, w)
        arr = np.stack([arr, arr, arr], axis=-1)
    elif enc == "16uc1":
        # realsense depth: 16-bit mm. Normalize to 8-bit for visualization.
        raw = np.frombuffer(msg.data, dtype=np.uint16).reshape(h, w)
        arr = (raw / raw.max() * 255).astype(np.uint8) if raw.max() > 0 else np.zeros((h, w), np.uint8)
        arr = np.stack([arr, arr, arr], axis=-1)
    elif enc == "32fc1":
        raw = np.frombuffer(msg.data, dtype=np.float32).reshape(h, w)
        valid = np.isfinite(raw)
        if valid.any():
            mn, mx = raw[valid].min(), raw[valid].max()
            norm = np.where(valid, (raw - mn) / max(mx - mn, 1e-6) * 255, 0).astype(np.uint8)
        else:
            norm = np.zeros((h, w), np.uint8)
        arr = np.stack([norm, norm, norm], axis=-1)
    else:
        raise ValueError(f"unsupported image encoding: {enc}")
    from PIL import Image as PILImage
    buf = BytesIO()
    PILImage.fromarray(np.ascontiguousarray(arr)).save(buf, format="JPEG", quality=85)
    return buf.getvalue()


def _on_rgb(msg) -> None:
    global _latest_rgb_jpeg, _rgb_frame_id
    try:
        jpg = _ros_image_to_jpeg(msg)
        with _state_lock:
            _latest_rgb_jpeg = jpg
            if msg.header.frame_id:
                _rgb_frame_id = msg.header.frame_id
    except Exception as e:  # noqa: BLE001
        log.warning("RGB conversion error: %s", e)


def _on_depth(msg) -> None:
    global _latest_depth_jpeg, _depth_frame_id
    try:
        jpg = _ros_image_to_jpeg(msg)
        with _state_lock:
            _latest_depth_jpeg = jpg
            if msg.header.frame_id:
                _depth_frame_id = msg.header.frame_id
    except Exception as e:  # noqa: BLE001
        log.warning("depth conversion error: %s", e)


# ── MCP snapshot tools (typed against codegen MCP dataclasses) ──────────────
import builtin_interfaces_mcp  # noqa: E402
import std_msgs_mcp  # noqa: E402
from sensor_msgs_mcp import Image  # noqa: E402
from std_msgs_mcp import Empty  # noqa: E402


def _now_header(frame_id: str) -> std_msgs_mcp.Header:
    now = time.time()
    sec = int(now)
    ns = int((now % 1) * 1e9) % 1_000_000_000
    return std_msgs_mcp.Header(
        stamp=builtin_interfaces_mcp.Time(sec=sec, nanosec=ns),
        frame_id=frame_id,
    )


def _jpeg_to_image_mcp(jpg: bytes, frame_id: str) -> Image:
    from PIL import Image as PILImage
    im = PILImage.open(BytesIO(jpg))
    w, h = im.size
    return Image(
        header=_now_header(frame_id),
        height=h, width=w,
        encoding="jpeg",
        is_bigendian=0,
        step=len(jpg),
        data=jpg,
    )


def _empty_image_error(reason: str) -> Image:
    """Return a tiny black 1x1 JPEG when we can't deliver a frame.
    Reason is encoded in frame_id so the agent can read it."""
    from PIL import Image as PILImage
    buf = BytesIO()
    PILImage.new("RGB", (1, 1), (0, 0, 0)).save(buf, format="JPEG")
    return _jpeg_to_image_mcp(buf.getvalue(), f"error:{reason}")


@cap.mcp("robonix/primitive/camera/snapshot")
def snapshot(msg: Empty) -> Image:
    """PRIMARY perception tool. Use freely — between every chassis/cmd
    burst — to see what's in front of the robot and decide what to do
    next. Returns the current RGB frame as a JPEG-encoded
    sensor_msgs/Image (encoding='jpeg', data=JPEG bytes)."""
    with _state_lock:
        data = _latest_rgb_jpeg
        frame_id = _rgb_frame_id
    if data is None:
        return _empty_image_error("no RGB frame received yet")
    return _jpeg_to_image_mcp(data, frame_id)


@cap.mcp("robonix/primitive/camera/depth_snapshot")
def depth_snapshot(msg: Empty) -> Image:
    """Depth snapshot as 8-bit JPEG (normalized for visualization).
    Returns sensor_msgs/Image with encoding='jpeg'. For actual metric
    depth, subscribe to robonix/primitive/camera/depth (16UC1)."""
    with _state_lock:
        data = _latest_depth_jpeg
        frame_id = _depth_frame_id
    if data is None:
        return _empty_image_error("no depth frame received yet")
    return _jpeg_to_image_mcp(data, frame_id)


def _publish_extrinsics_when_ready(
    base_frame: str, camera_frame: str, topic: str,
) -> None:
    """Publish the static robot-to-camera TF as the Atlas extrinsics contract.

    TF may come up after the camera process, so this waits in a daemon thread.
    Failure is non-fatal: RGB, depth, and snapshot capabilities still run.
    """
    try:
        from rclpy.duration import Duration  # type: ignore
        from rclpy.time import Time  # type: ignore
        from tf2_ros import Buffer, TransformListener  # type: ignore
        from robonix_api.ros import RosBackend

        node = RosBackend.get().node
        buffer = Buffer()
        TransformListener(buffer, node)
        deadline = time.monotonic() + 60.0
        while time.monotonic() < deadline:
            try:
                transform = buffer.lookup_transform(
                    base_frame, camera_frame, Time(), Duration(seconds=0.5),
                )
            except Exception:  # noqa: BLE001
                time.sleep(0.5)
                continue
            transform.header.frame_id = base_frame
            transform.child_frame_id = camera_frame
            publisher = _extrinsics_pub
            if publisher is not None:
                publisher.publish(transform)
                t = transform.transform.translation
                log.info(
                    "published camera extrinsics %s -> %s on %s "
                    "(%.3f, %.3f, %.3f)",
                    base_frame, camera_frame, topic, t.x, t.y, t.z,
                )
            return
        log.warning(
            "camera extrinsics unavailable after 60s; TF chain %s -> %s "
            "was not resolvable",
            base_frame, camera_frame,
        )
    except Exception as exc:  # noqa: BLE001
        log.warning("camera extrinsics publisher unavailable: %s", exc)


# ── lifecycle ────────────────────────────────────────────────────────────────
@cap.on_init
def init(cfg: dict):
    """REGISTERED → INACTIVE: start streams and publish camera contracts."""
    global _extrinsics_pub
    cam = cfg.get("camera_name", "camera_435i")
    rgb_topic = cfg.get("rgb_topic", f"/{cam}/color/image_raw")
    depth_topic = cfg.get(
        "depth_topic", f"/{cam}/aligned_depth_to_color/image_raw"
    )
    intrinsics_topic = cfg.get("intrinsics_topic", f"/{cam}/color/camera_info")
    extrinsics_topic = cfg.get("extrinsics_topic", "/camera/extrinsics")
    extrinsics_base_frame = cfg.get("extrinsics_base_frame", "base_link")
    extrinsics_camera_frame = cfg.get(
        "extrinsics_camera_frame", "camera_color_optical_frame",
    )
    sentinel_timeout = float(cfg.get("sentinel_timeout_s", 30.0))

    try:
        _spawn_realsense(cfg)
    except Exception as e:  # noqa: BLE001
        log.warning("spawn realsense failed; continuing without camera: %s", e)

    # Subscribe RGB + depth via robonix_api (declare=False — we declare
    # the ros2 topic_out interfaces explicitly below, after sentinel passes).
    cap.create_subscription(
        "robonix/primitive/camera/rgb",
        topic=rgb_topic, msg_type="Image",
        callback=_on_rgb, qos="best_effort", declare=False,
    )
    cap.create_subscription(
        "robonix/primitive/camera/depth",
        topic=depth_topic, msg_type="Image",
        callback=_on_depth, qos="best_effort", declare=False,
    )

    # Do not gate SOMA initialization on camera availability.  The camera may
    # be disconnected or slow to start; keep the process and topic contracts
    # alive so the rest of the robot can start, and report the condition.
    if not cap.wait_for_topic(rgb_topic, "Image", sentinel_timeout):
        log.warning(
            "no Image on %s within %.1fs; continuing without camera frames",
            rgb_topic, sentinel_timeout,
        )

    cap.declare_ros2_topic(
        "robonix/primitive/camera/rgb",
        topic=rgb_topic, qos="best_effort",
    )
    cap.declare_ros2_topic(
        "robonix/primitive/camera/depth",
        topic=depth_topic, qos="best_effort",
    )
    # Pinhole intrinsics (sensor_msgs/CameraInfo) for the color stream. Depth is
    # aligned_depth_to_color, so consumers reuse the color K to back-project.
    # Without this, scene's ConceptGraphs detector blocks forever on
    # "waiting for camera intrinsics" and never produces 3D objects.
    cap.declare_ros2_topic(
        "robonix/primitive/camera/intrinsics",
        topic=intrinsics_topic,
        qos="reliable",
    )
    from geometry_msgs.msg import TransformStamped  # type: ignore
    _extrinsics_pub = cap.create_publisher(
        "robonix/primitive/camera/extrinsics",
        topic=extrinsics_topic,
        msg_type=TransformStamped,
        qos="latched",
    )
    threading.Thread(
        target=_publish_extrinsics_when_ready,
        args=(extrinsics_base_frame, extrinsics_camera_frame, extrinsics_topic),
        daemon=True,
    ).start()
    log.info(
        "init complete: rgb=%s depth=%s intrinsics=%s extrinsics=%s "
        "(%s -> %s) + snapshot/depth_snapshot MCP exposed",
        rgb_topic, depth_topic, intrinsics_topic, extrinsics_topic,
        extrinsics_base_frame, extrinsics_camera_frame,
    )
    return Ok()


@cap.on_shutdown
def shutdown():
    global _extrinsics_pub
    _kill_realsense()
    _extrinsics_pub = None
    return Ok()


if __name__ == "__main__":
    cap.run()
