#!/usr/bin/env python3
# SPDX-License-Identifier: MulanPSL-2.0
"""mid360_lidar_rbnx — Livox MID-360 lidar primitive (capability_id=mid360_lidar).

Owns `robonix/primitive/lidar/*`. The MID-360 also publishes `/livox/imu`
as a side-effect of the same upstream launch, but the IMU contract
surface lives in a SEPARATE package (`mid360_imu_rbnx`) per the
"one primitive namespace = one package" invariant.

Lifecycle:
    on_init  — parse cfg → spawn livox launch → wait for first PointCloud2
               → declare ros2 topic_out for primitive/lidar/lidar3d.
    on_shutdown — kill livox subprocess.

Config (from manifest's `config:` block, delivered via Driver(CMD_INIT)):
    lidar_topic        default "/scanner/cloud"  (matches our lddc.cpp patch)
    lidar_ip           default 192.168.1.161
    host_ip            default auto from `ip route get <lidar_ip>`
    xfer_format        default 0  (ROS2 PointCloud2 PointXYZRTLT)
    publish_freq       default 10.0
    frame_id           default "livox_frame"  (lidar's own frame on /tf)
    parent_frame       default "base_link"    (body frame for the STP)
    extrinsics         optional 6-DoF mount pose of the lidar in
                       parent_frame. Shape: {x, y, z, roll, pitch, yaw}
                       (radians). When present we spawn a
                       static_transform_publisher parent_frame → frame_id
                       so consumers (mapping etc.) see a complete TF
                       tree without needing chassis or soma. Skip when
                       a chassis driver / soma URDF already publishes
                       the same edge — double-publishing TFs causes
                       'TF_REPEATED_DATA ignoring data' warnings.
    sentinel_timeout_s default 30.0
"""
from __future__ import annotations

import json
import logging
import os
import signal
import subprocess
import threading
import time
from pathlib import Path

from robonix_api import Primitive, Ok, Err

logging.basicConfig(
    level=os.environ.get("MID360_LOG_LEVEL", "INFO"),
    format="[mid360] %(message)s",
)
log = logging.getLogger("mid360")

cap = Primitive(id="mid360_lidar", namespace="robonix/primitive/lidar")


def _pump_output(stream, tag: str) -> None:
    """Forward a child process's merged stdout/stderr into scribe via the
    package logger — one unified log stream, no side-car *.log file."""
    for raw in iter(stream.readline, b""):
        line = raw.decode(errors="replace").rstrip()
        if line:
            log.info("[%s] %s", tag, line)

_pkg_root: Path = Path(__file__).resolve().parent.parent
_livox_proc: subprocess.Popen | None = None
_stp_proc: subprocess.Popen | None = None


# ── livox subprocess management ──────────────────────────────────────────
def _resolve_livox_config(cfg: dict) -> str:
    """Generate a Livox MID360_config.json with the right host_net_info.
    Returns the absolute path to the JSON to feed the upstream launch."""
    src_cfg = _pkg_root / "src" / "livox_ros_driver2" / "config" / "MID360_config.json"
    if not src_cfg.is_file():
        raise RuntimeError(f"packaged config missing: {src_cfg}")

    lidar_ip = str(cfg.get("lidar_ip") or os.environ.get("LIVOX_LIDAR_IP") or "")
    if not lidar_ip:
        try:
            data = json.loads(src_cfg.read_text())
            lidar_ip = data["lidar_configs"][0]["ip"]
        except Exception:  # noqa: BLE001
            lidar_ip = "192.168.1.161"

    host_ip = str(cfg.get("host_ip") or os.environ.get("LIVOX_HOST_IP") or "")
    if not host_ip:
        try:
            out = subprocess.run(
                ["ip", "-4", "route", "get", lidar_ip],
                capture_output=True, text=True, timeout=2, check=False,
            )
            parts = out.stdout.split()
            if "src" in parts:
                host_ip = parts[parts.index("src") + 1]
        except Exception:  # noqa: BLE001
            pass

    if not host_ip:
        log.warning(
            "could not resolve host IP (set host_ip in config or LIVOX_HOST_IP); "
            "using packaged JSON %s", src_cfg,
        )
        return str(src_cfg)

    out_path = _pkg_root / "rbnx-build" / "data" / "MID360_config.gen.json"
    out_path.parent.mkdir(parents=True, exist_ok=True)
    data = json.loads(src_cfg.read_text())
    for key in ("cmd_data_ip", "push_msg_ip", "point_data_ip", "imu_data_ip"):
        data["MID360"]["host_net_info"][key] = host_ip
    if cfg.get("lidar_ip"):
        data["lidar_configs"][0]["ip"] = lidar_ip
    out_path.write_text(json.dumps(data, indent=2))
    log.info("livox config: lidar=%s host=%s → %s", lidar_ip, host_ip, out_path)
    return str(out_path)


def _spawn_livox(cfg: dict) -> None:
    global _livox_proc
    config_path = _resolve_livox_config(cfg)
    env = dict(os.environ)
    env["LIVOX_MID360_CONFIG"] = config_path
    env["LIVOX_XFER_FORMAT"] = str(cfg.get("xfer_format", 0))
    env["LIVOX_PUBLISH_FREQ"] = str(cfg.get("publish_freq", 10.0))
    env["LIVOX_FRAME_ID"] = str(cfg.get("frame_id", "livox_frame"))

    log.info("spawning livox driver (xfer_format=%s)", env["LIVOX_XFER_FORMAT"])
    _livox_proc = subprocess.Popen(
        ["ros2", "launch", "livox_ros_driver2", "msg_MID360_launch.py"],
        env=env,
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
        start_new_session=True,
    )
    threading.Thread(target=_pump_output, args=(_livox_proc.stdout, "livox"),
                     daemon=True).start()


def _kill_livox() -> None:
    p = _livox_proc
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


# ── static_transform_publisher: parent_frame → frame_id ──────────────────
# Owned by the lidar primitive because the lidar is the sensor that
# *knows* its own frame id; `extrinsics` in cfg is the mount pose
# declared by whoever assembled the robot (deploy manifest). With
# A: primitive-publishes-its-own-static-TF, mapping never has to
# learn vendor frame names — every consumer sees a complete tree
# rooted at base_link.
def _spawn_stp(cfg: dict) -> None:
    global _stp_proc
    ext = cfg.get("extrinsics")
    if not ext:
        log.info("no extrinsics in cfg; assuming chassis/soma publishes "
                 "parent_frame → frame_id elsewhere")
        return
    parent = str(cfg.get("parent_frame", "base_link"))
    child = str(cfg.get("frame_id", "livox_frame"))
    args = [
        "ros2", "run", "tf2_ros", "static_transform_publisher",
        "--x", str(float(ext.get("x", 0.0))),
        "--y", str(float(ext.get("y", 0.0))),
        "--z", str(float(ext.get("z", 0.0))),
        "--roll", str(float(ext.get("roll", 0.0))),
        "--pitch", str(float(ext.get("pitch", 0.0))),
        "--yaw", str(float(ext.get("yaw", 0.0))),
        "--frame-id", parent,
        "--child-frame-id", child,
    ]
    log.info("spawning static_transform_publisher %s → %s @ %s",
             parent, child, ext)
    _stp_proc = subprocess.Popen(
        args, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
        start_new_session=True,
    )
    threading.Thread(target=_pump_output, args=(_stp_proc.stdout, "stp"),
                     daemon=True).start()


def _kill_stp() -> None:
    p = _stp_proc
    if p is None or p.poll() is not None:
        return
    try:
        os.killpg(os.getpgid(p.pid), signal.SIGTERM)
    except ProcessLookupError:
        return
    try:
        p.wait(timeout=2.0)
    except subprocess.TimeoutExpired:
        try:
            os.killpg(os.getpgid(p.pid), signal.SIGKILL)
        except ProcessLookupError:
            pass


# ── sentinel: wait for first PointCloud2 ─────────────────────────────────
def _wait_for_pointcloud(topic: str, timeout_s: float) -> bool:
    try:
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
        from sensor_msgs.msg import PointCloud2
    except ImportError as e:
        log.warning("rclpy unavailable (%s); skipping sentinel wait", e)
        return True
    rclpy.init(args=None)
    node = Node("mid360_atlas_sentinel")
    qos = QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
    )
    seen = threading.Event()
    node.create_subscription(PointCloud2, topic, lambda _m: seen.set(), qos)
    log.info("waiting for first PointCloud2 on %s — up to %.1fs", topic, timeout_s)
    deadline = time.monotonic() + timeout_s
    try:
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
            if seen.is_set():
                break
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:  # noqa: BLE001
            pass
    return seen.is_set()


# ── lifecycle handlers ───────────────────────────────────────────────────
@cap.on_init
def init(cfg: dict):
    """REGISTERED → INACTIVE: spawn livox, wait for cloud, declare topic.

    Self-heal: the MID-360 + livox_ros_driver2 intermittently completes
    the control-channel handshake (set work-mode / enable imu) but never
    starts the point-data UDP stream — the well-known "connected but not
    sampling" state that otherwise needs a manual driver/device restart.
    Detect it (no PointCloud2 within sentinel_timeout) and respawn the
    driver up to `livox_retries` times so a remote deploy recovers
    without anyone power-cycling the lidar.
    """
    lidar_topic = cfg.get("lidar_topic", "/scanner/cloud")
    sentinel_timeout = float(cfg.get("sentinel_timeout_s", 30.0))
    retries = int(cfg.get("livox_retries", 3))

    last_err = ""
    for attempt in range(1, retries + 1):
        try:
            _spawn_livox(cfg)
        except Exception as e:  # noqa: BLE001
            return Err(f"spawn livox failed: {e}")

        if _wait_for_pointcloud(lidar_topic, sentinel_timeout):
            if attempt > 1:
                log.info("livox point stream recovered on attempt %d/%d",
                         attempt, retries)
            break

        last_err = (f"no PointCloud2 on {lidar_topic} within "
                    f"{sentinel_timeout:.1f}s (attempt {attempt}/{retries})")
        log.warning("%s — respawning livox driver", last_err)
        _kill_livox()
    else:
        return Err(
            f"{last_err}; livox never started its point stream after "
            f"{retries} respawns — MID-360 may need a hardware power-cycle."
        )

    # parent_frame → frame_id static TF (no-op when extrinsics absent).
    try:
        _spawn_stp(cfg)
    except Exception as e:  # noqa: BLE001
        _kill_livox()
        return Err(f"spawn static_transform_publisher failed: {e}")

    cap.declare_ros2_topic(
        "robonix/primitive/lidar/lidar3d",
        topic=lidar_topic,
        qos="best_effort",
    )
    log.info("init complete: lidar3d=%s", lidar_topic)
    return Ok()


@cap.on_shutdown
def shutdown():
    _kill_stp()
    _kill_livox()
    return Ok()


if __name__ == "__main__":
    cap.run()
