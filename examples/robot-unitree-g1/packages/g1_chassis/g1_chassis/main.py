#!/usr/bin/env python3
"""g1_chassis — Unitree G1 chassis provider (SDK daemon + ROS2 adapter).

Starts the G1 SDK daemon and ROS2 adapter node, then registers with atlas
as a primitive offering chassis/driver, twist_in and odom capabilities.
"""
from __future__ import annotations

import logging
import math
import os
import signal
import subprocess
import time
from pathlib import Path

from robonix_api import Deferred, Err, Ok, Primitive

logging.basicConfig(
    level=os.environ.get("G1_CHASSIS_LOG_LEVEL", "INFO"),
    format="[g1_chassis] %(message)s",
)
log = logging.getLogger("g1_chassis")

g1_chassis = Primitive(
    id="g1_chassis", namespace="robonix/primitive/chassis"
)

_package_root = Path(
    os.environ.get(
        "RBNX_PACKAGE_ROOT", str(Path(__file__).resolve().parents[1])
    )
).resolve()

_adapter_binary = (
    _package_root
    / "rbnx-build"
    / "ros"
    / "install"
    / "lib"
    / "g1_chassis_adapter"
    / "g1_chassis_adapter_node"
)
_daemon_binary = (
    _package_root
    / "rbnx-build"
    / "sdk"
    / "install"
    / "bin"
    / "g1_loco_daemon"
)

_processes: list[subprocess.Popen] = []


def _is_executable(path: Path) -> bool:
    return path.is_file() and os.access(path, os.X_OK)


def _stop_processes() -> None:
    """Stop adapter first, then daemon (reverse order)."""
    ordered = list(reversed(_processes))
    for process in ordered:
        if process.poll() is not None:
            continue
        try:
            os.killpg(os.getpgid(process.pid), signal.SIGTERM)
        except ProcessLookupError:
            pass
    for process in ordered:
        if process.poll() is not None:
            continue
        try:
            process.wait(timeout=3.0)
        except subprocess.TimeoutExpired:
            try:
                os.killpg(os.getpgid(process.pid), signal.SIGKILL)
            except ProcessLookupError:
                pass
    _processes.clear()


def _as_bool(value) -> bool:
    """Parse manifest and environment boolean values consistently."""
    if isinstance(value, bool):
        return value
    text = str(value).strip().lower()
    if text in {"1", "true", "yes", "on"}:
        return True
    if text in {"0", "false", "no", "off"}:
        return False
    raise ValueError(f"invalid boolean value: {value!r}")


def _daemon_argv(socket_path: str, config: dict) -> list[str]:
    """Build the SDK daemon command from the active deployment config."""
    network_interface = str(
        config.get("network_interface", os.environ.get("G1_NETWORK_INTERFACE", ""))
    ).strip()
    allow_motion = _as_bool(
        config.get("allow_motion", os.environ.get("G1_ALLOW_MOTION", "false"))
    )
    argv = [
        str(_daemon_binary),
        "--socket", socket_path,
        "--watchdog-ms", "300",
        "--max-vx", str(config.get("max_linear_x_mps", 0.35)),
        "--max-vy", str(config.get("max_linear_y_mps", 0.20)),
        "--max-wz", str(config.get("max_angular_z_rps", 0.60)),
    ]
    if network_interface:
        argv.extend(["--interface", network_interface])
    if not network_interface:
        raise ValueError("network_interface is required for real G1 odometry")
    argv.extend(["--state-topic", str(config.get("state_topic", "rt/odommodestate")).strip()])
    if allow_motion:
        argv.extend(["--allow-motion", "--motion-ack", "G1_PHYSICAL_MOTION_APPROVED"])
    return argv


def _daemon_env(socket_path: str) -> dict[str, str]:
    """Keep the private SDK DDS libraries scoped to the daemon process."""
    env = os.environ.copy()
    env["G1_IPC_SOCKET"] = socket_path
    sdk_lib = str(_package_root / "rbnx-build" / "sdk" / "install" / "lib")
    env["LD_LIBRARY_PATH"] = sdk_lib + ":" + env.get("LD_LIBRARY_PATH", "")
    return env


def _adapter_argv(config) -> list[str]:
    """Build ROS2 adapter arguments from provider config."""
    return [
        str(_adapter_binary), "--ros-args",
        "-p", f"twist_in_topic:={config.get('twist_in_topic', '/cmd_vel')}",
        "-p", f"odom_topic:={config.get('odom_topic', '/odom')}",
        "-p", f"publish_odom_tf:={str(_as_bool(config.get('publish_odom_tf', True))).lower()}",
        "-p", f"odom_frame:={config.get('odom_frame', 'odom')}",
        "-p", f"base_frame:={config.get('base_frame', 'base_footprint')}",
        "-p", f"joint_state_topic:={config.get('joint_state_topic', '/joint_states')}",
        "-p", f"odom_publish_rate_hz:={float(config.get('odom_publish_rate_hz', 50.0))}",
        "-p", f"state_timeout_s:={float(config.get('state_timeout_s', 0.15))}",
        "-p", "use_sim_time:=false",
    ]


def _validate_config(config: dict) -> None:
    """Reject unsupported odometry settings and invalid limits before starting processes."""
    if config.get("odom_source", "sdk_pose") != "sdk_pose":
        raise ValueError("odom_source must be sdk_pose")
    topic = str(config.get("state_topic", "rt/odommodestate")).strip()
    if not topic.startswith("rt/") or len(topic) <= 3 or any(char.isspace() for char in topic):
        raise ValueError("state_topic must be a nonempty rt/ DDS topic")
    _as_bool(config.get("publish_odom_tf", True))
    for key, default, minimum, maximum in (
        ("odom_publish_rate_hz", 50.0, 10.0, 100.0),
        ("state_timeout_s", 0.15, 0.05, 0.30),
        ("max_linear_x_mps", 0.35, 0.0, 1.0),
        ("max_linear_y_mps", 0.20, 0.0, 0.5),
        ("max_angular_z_rps", 0.60, 0.0, 2.0),
    ):
        value = float(config.get(key, default))
        if not math.isfinite(value) or not minimum <= value <= maximum:
            raise ValueError(f"{key} must be between {minimum} and {maximum}")


@g1_chassis.on_init
def initialize(config):
    """Start daemon + adapter, wait for odometry, then declare topics."""
    global _processes

    _stop_processes()

    socket_path = os.environ.get(
        "G1_IPC_SOCKET",
        str(Path.home() / ".robonix" / "g1_chassis.sock"),
    )
    try:
        _validate_config(config)
        daemon_argv = _daemon_argv(socket_path, config)
        adapter_argv = _adapter_argv(config)
    except (TypeError, ValueError) as error:
        return Err(f"invalid G1 chassis config: {error}")
    if not Path(socket_path).is_absolute() or len(os.fsencode(socket_path)) >= 108:
        return Err("G1_IPC_SOCKET must be absolute and fit sockaddr_un")
    try:
        os.makedirs(os.path.dirname(socket_path), exist_ok=True)
    except OSError as error:
        return Err(f"cannot create G1 socket directory: {error}")

    if not _is_executable(_adapter_binary):
        return Err(f"adapter not built: {_adapter_binary}")
    if not _is_executable(_daemon_binary):
        return Err(f"daemon not built: {_daemon_binary}")

    # Start the SDK daemon with the deployment's explicit motion gate.
    daemon_env = _daemon_env(socket_path)

    try:
        _processes.append(
            g1_chassis.spawn(
                daemon_argv,
                env=daemon_env,
                log="sdk-daemon.log",
                cwd=str(_package_root),
            )
        )
    except Exception as error:
        _stop_processes()
        return Err(f"failed to start G1 SDK daemon: {error}")

    # Wait for daemon to bind the socket (up to 3s)
    deadline = time.monotonic() + 3.0
    while time.monotonic() < deadline:
        if os.path.exists(socket_path):
            break
        time.sleep(0.1)

    if not os.path.exists(socket_path):
        _stop_processes()
        return Err("g1_loco_daemon did not create socket within 3s")

    if _processes[0].poll() is not None:
        code = _processes[0].returncode
        _stop_processes()
        return Err(f"g1_loco_daemon exited with code {code}")

    # Start ROS2 adapter (daemon env already has G1_IPC_SOCKET)
    try:
        _processes.append(
            g1_chassis.spawn(
                adapter_argv,
                env={**os.environ, "G1_IPC_SOCKET": socket_path},
                log="adapter.log",
                cwd=str(_package_root),
            )
        )
    except Exception as error:
        _stop_processes()
        return Err(f"failed to start G1 ROS adapter: {error}")

    # Wait for adapter ROS topics to appear (up to 10s)
    try:
        odom_available = g1_chassis.wait_for_topic(
            config.get("odom_topic", "/odom"),
            "nav_msgs/msg/Odometry",
            10.0,
        )
    except Exception as error:
        _stop_processes()
        return Err(f"wait_for_topic failed: {error}")

    if not odom_available:
        code = _processes[-1].poll()
        _stop_processes()
        if code is not None:
            return Err(f"adapter exited during startup with code {code}")
        return Deferred("No real G1 odometry received within 10s; check SDK state and adapter diagnostics")

    # Check adapter didn't crash
    if _processes[-1].poll() is not None:
        code = _processes[-1].returncode
        _stop_processes()
        return Err(f"adapter exited with code {code}")

    # Declare ROS2 topic capabilities with atlas.
    # The *driver capability is auto-declared by _do_bootstrap() with
    # Transport.GRPC — no manual declaration needed here.
    try:
        g1_chassis.declare_ros2_topic(
            "robonix/primitive/chassis/twist_in",
            config.get("twist_in_topic", "/cmd_vel"),
            qos="reliable",
        )
        g1_chassis.declare_ros2_topic(
            "robonix/primitive/chassis/odom",
            config.get("odom_topic", "/odom"),
            qos="reliable",
        )
    except Exception as error:
        _stop_processes()
        return Err(f"failed to declare chassis contracts: {error}")

    log.info(
        "G1 chassis initialized "
        "(daemon pid=%d, adapter pid=%d)",
        _processes[0].pid,
        _processes[-1].pid,
    )
    return Ok()


@g1_chassis.on_shutdown
def shutdown():
    """Stop adapter and daemon."""
    _stop_processes()
    return Ok()


if __name__ == "__main__":
    g1_chassis.run()
