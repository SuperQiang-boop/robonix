# G1 Chassis Primitive

Guarded Unitree G1 chassis adapter using the SDK2 LocoClient.

## Architecture

```
[ROS2 /cmd_vel] → g1_chassis_adapter_node → IPC socket → g1_loco_daemon → SDK2 LocoClient → G1 robot
```

### Components

| Component | Language | Purpose |
|---|---|---|
| `g1_loco_daemon` | C++ | IPC server wrapping SDK2 LocoClient. 300 ms watchdog, velocity clamping, motion gating. |
| `g1_chassis_adapter_node` | C++ (ROS2) | Subscribes to `/cmd_vel`, forwards commands to daemon via Unix socket, and publishes odom plus neutral joint-state heartbeats. |
| `g1_chassis/main.py` | Python | Robonix primitive provider — spawns adapter + daemon, declares ROS2 capabilities. |

### Safety

- **Motion disabled by default** — daemon rejects all velocity commands unless `--allow-motion` is passed at daemon startup, which requires the `G1_PHYSICAL_MOTION_APPROVED` acknowledgement.
- **300 ms watchdog** — if no valid cmd_vel arrives within 300 ms, the daemon issues StopMove and faults.
- **Velocity limits** — vx, vy, omega are clamped to configured maximums.
- **Zero-velocity stop** — the first all-zero cmd_vel after a movement command issues `StopMove`; repeated zeros do not flood the SDK.
- **Adapter disconnect** — if the IPC peer disconnects, the daemon immediately issues StopMove.

### ROS outputs

- `/odom` carries the stationary odometry heartbeat used by mapping and navigation while real G1 odometry is not integrated.
- `/joint_states` carries all 29 movable joint positions at zero so `robot_state_publisher` can connect `base_link` to both legs, both arms, and torso-mounted sensors. This is a visualization placeholder, not measured robot posture. Velocity and effort are omitted because they are unknown. Replace this placeholder publisher when integrating actual SDK joint feedback; do not publish competing positions for the same joints.

### IPC Protocol

- **Socket**: Unix stream socket, path set by `G1_IPC_SOCKET` env var.
- **Command packet** (24 bytes): type (1) + sequence (1) + reserved (6) + vx (4) + vy (4) + omega (4) — velocities are fixed-point ×10000.
- **Reply packet** (16 bytes): type (1) + sequence (1) + code (1) + armed (1) + faulted (1).

## Building

```bash
export UNITREE_SDK2_DIR=/path/to/unitree_sdk2-main
bash scripts/build.sh
```

### TF smoke test

With ROS sourced, run this in an isolated ROS domain. It uses a fake IPC peer and does not start the SDK daemon. It checks all movable joints against the deployment URDF and verifies every link has a transform to `base_link`.

```bash
ROS_DOMAIN_ID=217 ROS_LOCALHOST_ONLY=1 /usr/bin/python3 tests/tf_smoke.py \
  rbnx-build/ros/install/lib/g1_chassis_adapter/g1_chassis_adapter_node \
  ../../g1_description/g1_29dof.urdf
```

## Runtime

The provider (`main.py`) is launched by the Robonix primitive engine. It resolves:

- `G1_NETWORK_INTERFACE` — ethernet interface to the G1 (default: `lo`)
- `G1_ALLOW_MOTION` — `"true"` to enable motion (default: `false`)

Navigation TF uses `odom -> base_footprint` (configured by `odom_frame` and
`base_frame`, passed to the ROS adapter). The URDF owns
`base_footprint -> base_link` at the nominal standing pelvis height of 0.55 m.
The adapter currently publishes stationary placeholder odometry, not measured
SDK odometry; its zero roll/pitch cannot provide walking gravity compensation.

With mapping gravity alignment enabled, `publish_odom_tf: false` suppresses
chassis TF; internal ICP owns `odom -> base_footprint`. The `/odom` heartbeat
remains placeholder data and is not used by mapping or navigation in this mode.
Navigation explicitly consumes `/rtabmap/odom`.

Before enabling motion, the operator must place the G1 in its intended
locomotion mode. The chassis adapter preserves that mode; it does not issue
`Start` or `BalanceStand` when it receives a velocity command.
