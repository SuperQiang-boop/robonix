---
description: Guarded Unitree G1 chassis topics with measured planar SDK odometry and no-motion default.
---

# Unitree G1 chassis capability

This package provides the standard Robonix chassis contracts used by the G1
example:

- `robonix/primitive/chassis/twist_in` maps to `/cmd_vel`.
- `robonix/primitive/chassis/odom` maps to `/odom`.
- `robonix/primitive/chassis/driver` manages provider registration and process
  lifecycle.

It intentionally does not expose posture control, stand, sit, arm control,
hand control, low-level joint control, or a single-shot movement RPC. Nav2, or
another explicitly configured velocity controller, is expected to be the only
source of velocity commands.

## Current behavior

The provider starts a ROS 2 adapter and an SDK2 daemon connected through a
same-user Unix `SOCK_SEQPACKET` socket. The adapter subscribes to the
configured velocity input topic, publishes measured `/odom`, publishes all 29
neutral joint placeholders, and broadcasts `odom -> base_footprint`. The URDF
owns `base_footprint -> base_link` at nominal standing pelvis height.

The daemon subscribes to `rt/odommodestate` independently of its motion gate.
The adapter projects measured position and quaternion onto planar odometry at
50 Hz. Twist is derived in the child frame from consecutive pose differences.
Timestamps represent local DDS reception, not acquisition. Only fresh, new,
valid samples publish; there is no synthetic odometry heartbeat. State older
than 150 ms inhibits command refresh and TF; pose discontinuities above 0.30 m
or 0.50 rad latch a fault until restart. Neutral joints remain visualization
data and must not be treated as measured posture.

The provider defaults to motion disabled; deployment `allow_motion` or
`G1_ALLOW_MOTION` can explicitly enable the gate. The daemon
can parse a motion-capable command line only when `--allow-motion`,
`--interface`, and the exact `--motion-ack G1_PHYSICAL_MOTION_APPROVED` value
are supplied together, with the audited `300 ms` watchdog.

## Safety boundaries

Motion-disabled operation does not initialize the Unitree SDK2 client and
therefore cannot issue `SetVelocity` or `StopMove` through SDK2. It still
starts a receive-only SDK state subscriber and ROS adapter. Chassis contracts
are declared only after a real odometry message; a 10 s startup timeout cleans
up the processes and returns Deferred.

When motion is explicitly enabled outside the current default startup path, the
daemon applies the following local checks:

- The socket path must be absolute, owned by the current user, and fit
  `sockaddr_un`.
- The IPC peer must have the same UID as the daemon process.
- Motion requires an explicit network interface and the exact physical-motion
  acknowledgement string.
- Motion-capable watchdog configuration must be exactly `300 ms`.
- Velocity commands are clamped by the configured `vx`, `vy`, and yaw-rate
  limits before reaching SDK2.
- Adapter disconnect, watchdog expiry, shutdown, SDK errors, and malformed IPC
  packets fail closed by stopping or refusing motion.

These checks are defense-in-depth only. They are not evidence that G1 physical
motion has been validated.

## Open integration limits

This G1 package does not currently implement the Go2 SportModeState health
chain, opaque firmware marker review, external verified odometry quarantine,
or staged Nav2 arming profile. Those Go2-specific gates must not be assumed for
G1.

Before any motion-capable G1 deployment, the provider startup path, velocity
forwarding, odometry source, operator approval flow, watchdog behavior, stop
behavior, and field test procedure need a fresh review on the target robot.
