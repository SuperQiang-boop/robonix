# G1 Python final approach

`final_approach_g1.py` ports the algorithm supplied in `/data/final_approach_g1.py`.
It is independent of `PersistentRotationShim` and `SettledGoalChecker`.
Nav2 requires C++ controller plugins, so `src/python_final_approach.cpp` supplies
a small adapter and matching goal checker. The algorithm runs in one private
Python subprocess; it does not publish ROS velocity or goal topics itself.

MPPI follows the route until the robot is within `takeover_dist: 0.60` m. Python
then owns final approach until success or navigation cancellation. Pose/goal
come from Nav2 in the path's frame (normally `map`), using Nav2's existing TF
and `base_footprint`. The original `/local_nav_status` Dock Machine trigger,
`/teb_polar_node/cmd_vel`, `base_link` lookup and raw-forwarding notifications
are not used. Commands return through Controller Server, velocity smoother,
velocity guard and the existing chassis `/cmd_vel` subscription. No second
velocity publisher competes with Nav2. No terminal costmap/obstacle logic is
added; existing downstream velocity limits and guard remain active.

Original body-frame P control, EMA, gait floors, distance hysteresis, per-cycle
slew and pose dwell remain. Unlike the first controller, this version can
command reverse/lateral correction and still requires position within 0.08 m,
yaw within 0.12 rad, continuously for 1 s. The uploaded script's dwell checks
pose, not measured stopped velocity. Forward/lateral caps change from
0.38/0.30 to 0.35/0.20 m/s for this deployment. Nav2's planar speed limit overrides
gait floors. Keep Controller Server at 20 Hz for the original per-cycle limits.
`enable_gait_min_vel` is a numeric 1.0/0.0 parameter in this adapter.

## Switch

In the robot's `robonix_manifest.yaml`, change only the nav2 service's
`config.params_file`:

- First controller: `config/nav2_params_g1.yaml` (current default).
- Second controller: `config/nav2_params_g1_python.yaml`.

Stop the current navigation before switching. Build with `rbnx build` from
`/data/robonix/examples/robot-unitree-g1`, then restart using the normal project
startup procedure to load the chosen file. The first build installs the Python
worker and new plugin library. This is a restart-time choice, not live hot
switching. Both controller and goal checker are selected together by each
profile. Scene, frontend commands, TF and chassis topic names remain unchanged.
If the shared navigation/velocity settings change later, update both profiles.

Offline check (no ROS or robot required):

```sh
python3 -m unittest discover -s services/service-navigation-rbnx -p test_python_final_approach.py
```

The C++ plugin loading test additionally covers both new plugin classes; it
requires a built ROS/Nav2 workspace. The Python subprocess never drives hardware
on its own, and exits on malformed/nonfinite requests. Worker errors/timeouts
raise a Nav2 controller error rather than falling back to another controller.
