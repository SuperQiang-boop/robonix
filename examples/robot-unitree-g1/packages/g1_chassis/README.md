# G1 Chassis Primitive

Guarded Unitree G1 chassis adapter using the SDK2 LocoClient.

## Architecture

```
[ROS2 /cmd_vel] → g1_chassis_adapter_node → IPC socket → g1_loco_daemon → SDK2 LocoClient → G1 robot
[G1 rt/odommodestate] → SDK daemon → versioned IPC state → ROS adapter → /odom + odom→base_footprint
```

### Components

| Component | Language | Purpose |
|---|---|---|
| `g1_loco_daemon` | C++ | SDK state subscriber and guarded LocoClient. 300 ms watchdog, velocity clamping, motion gating. |
| `g1_chassis_adapter_node` | C++ (ROS2) | Real planar odom/TF, guarded command forwarding, neutral joint-state visualization. |
| `g1_chassis/main.py` | Python | Robonix primitive provider — spawns adapter + daemon, declares ROS2 capabilities. |

### Safety

- **Motion disabled by default** — daemon rejects all velocity commands unless `--allow-motion` is passed at daemon startup, which requires the `G1_PHYSICAL_MOTION_APPROVED` acknowledgement.
- **300 ms watchdog** — if no valid cmd_vel arrives within 300 ms, the daemon issues StopMove and faults.
- **Velocity limits** — vx, vy, omega are clamped to configured maximums.
- **Zero-velocity stop** — the first all-zero cmd_vel after a movement command issues `StopMove`; repeated zeros do not flood the SDK.
- **Adapter disconnect** — if the IPC peer disconnects, the daemon immediately issues StopMove.
- **Bounded SDK calls** — LocoClient RPC timeout is 100 ms. Locomotion mode is read once before opening the command socket, not during the watchdog-controlled command loop. Velocity commands expire after the watchdog interval (300 ms), rather than requesting day-long motion. A velocity RPC error immediately requests `StopMove` and latches a fault; unconfirmed stops are retried, and later commands cannot automatically re-arm it.
- **No queued motion replay** — each command batch uses only the newest velocity, with an all-zero stop taking priority. A batch that reaches the drain limit stops instead of executing potentially old motion.

### Diagnosing a navigation stall

`rbnx-build/data/sdk-daemon.log` records successful `SetVelocity` calls at most
once per second during uninterrupted motion, plus motion starts, errors and slow
RPCs. Each record includes Unix wall time, call count, command sequence, RPC
result/duration, inter-command gap and the clamped SDK velocity. Every `StopMove`
records its cause (`zero_command`, `velocity_rpc_error`, `watchdog_expired`,
`adapter_disconnect` or an unconfirmed-stop retry).
The independent state thread logs reception sequence/rate/age, IPC send sequence,
position/net displacement, quaternion and raw reported velocity once per second,
even if samples stop arriving. `delta_valid=0` means there is no comparable prior
sample. Reception counters and ages describe local DDS delivery, not sensor
acquisition; RPC success alone does not prove physical movement. Compare these
wall times with navigation trajectory records before changing TF or motion modes.

### ROS outputs

- `/odom` carries measured SDK x/y and quaternion yaw, projected onto `base_footprint` with z/roll/pitch zero. Body-frame twist is derived from pose differences and filtered with a 0.10 s time constant; raw SDK velocity is not assumed to be body-frame.
- Odom and TF share the local DDS reception timestamp because robot source stamps are zero on the tested firmware. This does not measure sensor acquisition latency. Default output is 50 Hz, only for new valid samples; stale data is never re-stamped.
- A separate state-forwarding thread delivers fresh SDK samples even while a motion RPC waits or the motion watchdog is faulted. It owns a duplicate IPC descriptor and is joined before peer teardown; socket backpressure drops intermediate samples instead of blocking or re-stamping them.
- Missing/invalid state inhibits motion refresh and TF. Reception age above 150 ms stops output; a pose jump above 0.30 m or 0.50 rad latches a fault until restart after checking localization. Pose covariance is a conservative assumption, not firmware-reported uncertainty.
- Jump diagnostics include the elapsed reception time and previous/current planar poses. Long outages do not bypass the discontinuity protection or automatically reset localization.
- `/joint_states` carries all 29 movable joint positions at zero so `robot_state_publisher` can connect `base_link` to both legs, both arms, and torso-mounted sensors. This is a visualization placeholder, not measured robot posture. Velocity and effort are omitted because they are unknown. Replace this placeholder publisher when integrating actual SDK joint feedback; do not publish competing positions for the same joints.

### IPC Protocol

- **Socket**: Same-user Unix `SOCK_SEQPACKET`, path set by `G1_IPC_SOCKET`.
- **Command packet** (24 bytes): type + sequence + vx/vy/omega int32 (fixed-point ×10000) + 10 reserved bytes.
- **Reply packet** (16 bytes): type + sequence + code + 9 reserved bytes + armed + faulted + 2 reserved bytes.
- **State packet** (76 bytes, version 1): type/version/reserved, error code, sequence, local wall/monotonic reception times, position, quaternion wxyz, raw velocity and yaw rate. SDK types never cross into the ROS adapter's linkage.

## Building

```bash
export UNITREE_SDK2_DIR=/path/to/unitree_sdk2-main
bash scripts/build.sh
```

### Offline and ROS smoke tests

With ROS sourced, run this in an isolated ROS domain. It uses a fake IPC peer and does not start the SDK daemon. It checks all movable joints against the deployment URDF and verifies every link has a transform to `base_link`.
Use `/usr/bin/python3` for the Humble Python ABI. If the shell's configured RMW
is unavailable, explicitly select an installed one such as `rmw_fastrtps_cpp`.

```bash
/usr/bin/python3 tests/deployment_test.py
ROS_DOMAIN_ID=217 ROS_LOCALHOST_ONLY=1 /usr/bin/python3 tests/odometry_smoke.py \
  rbnx-build/ros/install/lib/g1_chassis_adapter/g1_chassis_adapter_node
ROS_DOMAIN_ID=217 ROS_LOCALHOST_ONLY=1 /usr/bin/python3 tests/tf_smoke.py \
  rbnx-build/ros/install/lib/g1_chassis_adapter/g1_chassis_adapter_node \
  ../../g1_description/g1_29dof.urdf
```

For a bounded hardware receive-only test, the following starts a daemon without
`--allow-motion`, publishes to diagnostic topics in an isolated ROS graph, and
disables odometry TF. It never creates a velocity publisher or LocoClient:

```bash
ROS_DOMAIN_ID=217 ROS_LOCALHOST_ONLY=1 /usr/bin/python3 tests/live_odometry_readonly.py \
  --interface enp7s0 --seconds 15 --output /tmp/g1-readonly-odom.json
```

## Runtime

The provider (`main.py`) is launched by the Robonix primitive engine. It resolves:

- `G1_NETWORK_INTERFACE` — explicit Ethernet interface to G1 (required; deployment config takes precedence)
- `G1_ALLOW_MOTION` — `"true"` to enable motion (default: `false`)

Navigation TF uses `odom -> base_footprint` (configured by `odom_frame` and
`base_frame`, passed to the ROS adapter). The URDF owns
`base_footprint -> base_link` at the nominal standing pelvis height configured in the URDF.
The adapter publishes measured planar odometry. Its zero roll/pitch and neutral
joint placeholders cannot provide walking gravity compensation or measured
sensor extrinsics. Verify axis directions, pose continuity and mounting geometry
on hardware before physical navigation.

The deployment binds mapping `sensor_providers.odom: g1_chassis` and explicitly
includes `odom` in `rtabmap_inputs`. Internal ICP odometry is then disabled:
chassis alone owns `odom -> base_footprint`, RTAB-Map owns `map -> odom`, and
Nav2 consumes `/odom`. Do not launch BotBrain/WK odometry broadcasters alongside
this deployment. The provider waits for a real odometry message before declaring
its contracts; no state within 10 s returns Deferred and cleans up processes.

See `config.spec` for configuration defaults and supported ranges. The daemon
state subscription works with motion disabled and never initializes LocoClient
in that mode. SDK private DDS libraries stay scoped to the daemon.

Before enabling motion, the operator must place the G1 in its intended
locomotion mode. The chassis adapter preserves that mode; it does not issue
`Start` or `BalanceStand` when it receives a velocity command.
