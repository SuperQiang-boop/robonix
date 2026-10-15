# service-navigation-rbnx

Robonix wrapper around a system-installed ROS 2 Nav2 stack. The service
discovers map, odometry, and lidar inputs through Atlas and exposes
`robonix/service/navigation/*` over gRPC and MCP.

For Fast DDS on a host with multiple network interfaces, export
`RMW_IMPLEMENTATION=rmw_fastrtps_cpp` and `FASTRTPS_DEFAULT_PROFILES_FILE`
before boot. The Docker launcher mounts that readable XML file read-only and
forwards its container path. An invalid path fails before startup; leaving
the variable unset preserves the image's default profile.

## Deployment config

Navigation behavior belongs to the robot deployment, not this provider. Each
robot must carry a complete Nav2 YAML and reference it with `params_file`:

```yaml
service:
  - name: nav2
    url: https://github.com/syswonder/service-navigation-rbnx
    branch: main
    config:
      params_file: config/nav2_params.yaml
      provider_ids:
        map: mapping
        odom: chassis
        scan: lidar
      dynamic_speed:
        max_linear_speed_mps: 0.3
        default_percentage: 75
        step_percentage: 20
        min_percentage: 20
```

Relative paths are resolved from the directory containing
`robonix_manifest.yaml`. `config/nav2_params.example.yml` is a neutral example,
not a robot profile. Copy it into the deploy repository and set the robot's
frames, footprint, velocity and acceleration limits, costmap layers, goal
tolerances, topics, and planner/controller plugins there.

For a 3D lidar, bind `scan_cloud` and declare the adapter explicitly:

```yaml
      params_file: config/nav2_params.yaml
      provider_ids:
        map: mapping
        odom: chassis
        scan_cloud: lidar3d
      scan_projection:
        enabled: true
        target_frame: base_link
        min_height_m: 0.1
        max_height_m: 1.5
        range_max_m: 12.0
```

Optional `bt_xml_file` points to a deploy-owned BehaviorTree XML. Existing
`params_profile` deployments remain supported and emit a migration warning;
new deployments should not use that field. See `config.spec` for every
accepted instance field and default.

The final velocity guard publishes to `/cmd_vel` by default for compatibility.
Set `config.velocity_output_topic` to a fully-qualified non-motion sink such as
`/robonix/nomotion/cmd_vel` while integrating a physical robot. The
`ROBONIX_VELOCITY_OUTPUT_TOPIC` environment variable is the fallback when the
config field is absent; an explicit empty, relative, or malformed topic fails
startup before the guard creates any ROS endpoint.

Dynamic speed config uses Nav2-compatible SI semantics.
`max_linear_speed_mps` is the actual hard planar limit
`sqrt(vx^2 + vy^2)` in m/s after considering both the selected controller's
`max_speed_xy` and stricter per-axis limits. The final velocity guard
independently enforces this linear limit. `default_percentage` scales that
maximum, so the example starts at `0.225 m/s`. `step_percentage` is an additive
number of percentage points. Angular constraints such as DWB's
`max_vel_theta` stay in the deploy-owned Nav2 YAML; a controller may
proportionally adjust its internal kinematics when it processes a Nav2 speed
limit, but Robonix does not expose a separate angular-speed policy here.

`adjust_speed` handles `faster`, `slower`, and `normal`;
`set_speed_limit` applies an explicit percentage; and `get_speed_limit` reads
the current and configured state. By default a mutation belongs to the
selected active run and automatically restores the session limit when that run
terminates. `persist=true` deliberately changes the provider-session limit
across navigation runs. None of these operations restart Navigation, cancel a
goal, or resubmit it.

## Runtime

The final velocity guard also bounds terminal wandering. After first entering
`guard_terminal_xy_m`, it retains a `guard_terminal_timeout_s` deadline even if
the controller is translating or replanning. Moving farther than twice that XY
window, or exceeding the deadline, latches zero output and requests cancellation
of the active goal. This is a failure stop, never an arrival-success shortcut;
only a new goal UUID resets the latch. Stationary-rotation limits still apply.

The G1 profile additionally uses `PersistentRotationShim` and
`SettledGoalChecker` from `terminal_controller/`. The shim holds zero until
measured speed remains below 0.04 m/s and 0.06 rad/s for 0.40 s, then owns
collision-checked zero-linear terminal turning across same-goal replans,
including when position drifts outside the 0.20 m arrival radius. It never
falls back to MPPI during terminal rotation. It aims for 0.12 rad before
commanding a stop; the arrival checker independently requires current XY/yaw
within 0.20 m/0.25 rad and continuously stopped measurements for 0.40 s.
If position remains outside arrival tolerance after the turn settles, or exceeds
0.28 m while turning, it admits one position correction per navigation UUID:
measured stop, face the goal bearing, measured stop, then collision-checked
forward steering toward an inner 0.10 m window. Limits are 0.20 m/s (also honoring
external speed caps), 0.40 m/s², 0.30 m commanded travel/net displacement and
6 seconds including alignment/stops. It stops again before the final heading.
Same-goal replans do not reset this attempt or budget. A second correction,
collision or divergent bearing latches zero. The original
0.40 m outer boundary, 15-second overall deadline and strict arrival tolerances
remain unchanged; no infinite correction loop or false success is introduced.
If command travel/time is exhausted but current XY is already within the
original arrival radius, stop correction and resume final heading rather than
requiring the preferred inner 0.10 m window. The strict goal checker must still
observe simultaneous XY/yaw/standstill tolerance for the full dwell; drifting
out again does not grant a second correction. Measured displacement exhaustion
and budget exhaustion outside the arrival radius still fail closed. Failure
reports distinguish time, commanded travel and measured displacement limits.

The controller publishes UUID-scoped phase/failure diagnostics on
`/robonix/navigation/terminal`. The velocity guard pauses only final-goal-yaw
progress checks during correction, never outer XY/time or global spin limits.
Before requesting cancellation it atomically writes `guard-<uuid>.json` into
the configured `trajectory_log_dir` (default `rbnx-build/data/trajectories`).
The result callback reads only the matching UUID: safety stops become `FAILED`
with `safety_stop` and actual XY/yaw errors, while explicit user cancellation
remains `CANCELED`. Normal success still requires Nav2's real successful result.
This preserves the failure cause for Pilot rather than masking an unfinished
goal; Pilot may still fail if bounded recovery cannot complete navigation.
Offline result/UUID regression: `PYTHONPATH=. python3 -m unittest test_guard_failure`.
The deployment's `packages/g1_chassis/tests/nav2_guard_smoke.py` exercises the
production ROS guard with a synthetic cancellation service in a network-isolated
container (`ROS_DOMAIN_ID=217`, `ROS_LOCALHOST_ONLY=1`); it checks exact UUIDs,
cached reports, persist-before-cancel ordering and zero output after a trip.

Rebuild the navigation image after changing these C++ plugins using
`RBNX_PACKAGE_ROOT="$PWD" bash scripts/build.sh` from this service directory.
The Docker build context is the service root, with runtime data excluded by
`.dockerignore`; the image installs the plugins under
`/opt/robonix_nav2_terminal`, sourced by the entrypoint. Native builds use
`rbnx-build/terminal_controller/install`, sourced by `scripts/start_native.sh`,
and also require `ros-humble-nav2-rotation-shim-controller`.
The isolated G1 regression in the deployment README tests drift plus replans,
strict stopped arrival, both turn directions, and ordinary MPPI transit without
connecting a chassis. It does not establish physical in-place turning accuracy.

At `Driver(CMD_INIT)`, the wrapper:

1. resolves the selected Atlas providers;
2. resolves and materializes the deployment-owned Nav2 YAML;
3. starts an optional PointCloud2-to-LaserScan adapter;
4. starts Nav2 and waits for the `navigate_to_pose` action server;
5. connects to Nav2's live `speed_limit` subscriber;
6. exposes navigate, status, cancel, and dynamic speed capabilities.

Missing required providers return `deferred`. Invalid config or a Nav2 startup
failure returns `error` and tears down every child process.

## Build and tests

Navigation generates only its Atlas MCP bindings on every deployment target.
It deliberately does not generate, build, or source a Robonix ROS 2 IDL
overlay: the provider talks to Nav2 through the ROS 2 interfaces supplied by
the selected Humble installation, while its own public capability transport is
gRPC/MCP. Jetson native builds source only the system ROS 2 installation and
the locally built terminal-controller plugin overlay.

```bash
bash scripts/build.sh
python3 -m unittest -v \
  test_configuration.py \
  test_runtime_integration.py \
  test_rotation_guard.py \
  test_scan_filter.py \
  test_speed_control.py \
  test_velocity_limit.py
```

Jetson native builds require ROS 2 Humble and Nav2 packages compatible with the
host JetPack image. Docker manifests remain available for simulator and CI
deployments.
