---
description: Velocity-based basic movement skill for Unitree G1 with MCP interface.
---

# G1 basic movement skill

This skill exposes two MCP tools for direct velocity-based locomotion:

- `robonix/skill/basic_movement/move` — drive with linear/angular velocity for a duration
- `robonix/skill/basic_movement/stop` — immediately zero all velocities

It publishes `geometry_msgs/TwistStamped` on `/cmd_vel` at 10 Hz to satisfy the
chassis adapter's 300 ms watchdog.  Velocity limits match the g1_chassis daemon
clamps (vx ≤ 1.0 m/s, vy ≤ 0.5 m/s, wz ≤ 2.0 rad/s).

## Supported commands

### `move`

| Parameter   | Type   | Range           | Description                     |
|-------------|--------|-----------------|---------------------------------|
| `vx`        | float  | [-1.0, 1.0]     | Forward/backward velocity (m/s) |
| `vy`        | float  | [-0.5, 0.5]     | Lateral velocity (m/s)          |
| `wz`        | float  | [-2.0, 2.0]     | Angular velocity (rad/s)        |
| `duration_s`| float  | [0.1, 10.0]     | How long to move (seconds)      |

Returns `accepted=true` when the movement completes, or `accepted=false` if
another movement is already in progress.

### `stop`

No parameters.  Immediately zeros all velocities and sets an internal stop flag
that interrupts any in-progress `move` call.

## Safety

- All velocities are clamped to the chassis limits before publishing.
- `stop()` takes effect within one publish cycle (≤100 ms).
- Only one `move` call can be active at a time; concurrent calls are rejected.
- The skill publishes zero velocity on deactivation.
