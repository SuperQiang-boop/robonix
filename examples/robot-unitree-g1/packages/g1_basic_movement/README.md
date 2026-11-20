# G1 Basic Movement Skill

Velocity-based locomotion skill for the Unitree G1 humanoid robot.  Provides
simple MCP tools for LLM/Pilot-driven movement control without Nav2 path
planning.

## Quick reference

| MCP tool                                   | Description                        |
|--------------------------------------------|------------------------------------|
| `robonix/skill/basic_movement/move`        | Move with vx/vy/wz for duration    |
| `robonix/skill/basic_movement/stop`        | Emergency stop                     |

## Examples

```
# Move forward at 0.3 m/s for 2 seconds
move(vx=0.3, vy=0.0, wz=0.0, duration_s=2.0)

# Turn left at 0.5 rad/s for 3 seconds
move(vx=0.0, vy=0.0, wz=0.5, duration_s=3.0)

# Strafe right at 0.2 m/s for 1.5 seconds
move(vx=0.0, vy=-0.2, wz=0.0, duration_s=1.5)

# Stop immediately
stop()
```

## Dependencies

- ROS 2 Humble (for `geometry_msgs`, `rclpy`)
- Robonix API (`robonix_api.Skill`)
