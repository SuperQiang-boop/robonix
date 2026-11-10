# service-navigation-rbnx

Robonix wrapper around a system-installed ROS 2 Nav2 stack. The service
discovers map, odometry, and lidar inputs through Src and exposes
`robonix/service/navigation/*` over gRPC and MCP.

When `ROBONIX_GREETING_STATE_PATH` is set, navigation admission, queued dispatch
and resume require `movement_allowed: true` in that local greeting snapshot.
Missing/malformed state rejects movement. Cancelling a paused run cannot bypass
the restriction; cancel and status remain available. New G1 goals also wait for
any previous navigation to become terminal, rather than treating cancellation
acceptance as completion. Unset the variable in other deployments to retain
their existing behavior. Rebuild/reload navigation together with the G1 greeting
skill, Planner, Executor and Gateway when enabling this mode.

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

The G1 profile uses `PersistentRotationShim` and `SettledGoalChecker` from
`terminal_controller/`. Turning waits for measured standstill and retains
ownership across replans. After XY entry it preserves the requested final yaw
without starting position correction. Outer XY, time and
collision limits remain active.

An independent Python terminal algorithm based on the supplied
`final_approach_g1.py` is selectable through `config/nav2_params_g1_python.yaml`.
See [its adaptation and switching instructions](terminal_controller/python/README.md).
The default profile continues to select the first controller.

The G1 deployment latches entry within 0.60 m of the Scene approach pose for
one navigation goal. It then commands rotation only, preserving the requested
final yaw, without chasing the approach point. G1 gait motion during turning
may still cause physical translation. Arrival requires yaw error within
40 degrees and measured standstill for 0.40 s; re-entry into 0.60 m is not required.
The 0.70 m outer XY limit, 60 s deadline and rotation footprint checks remain
active. The goal checker latches outer/deadline failures until a new goal.
Replans preserve entry and budgets. These tolerances also apply to RViz goals.
Legacy correction helpers/parameters remain in source but are not invoked by
this terminal control path. Inflation radius remains 0.25 m. The velocity guard
enters its terminal window at 0.60 m; the controller enforces the stricter
0.70 m outer limit. Success details explain that feedback distance need not be
zero and last_pose is the endpoint, not the starting pose.

The controller publishes UUID-scoped phase/failure diagnostics on
`/robonix/navigation/terminal`. The velocity guard pauses only final-goal-yaw
progress checks during correction, never outer XY/time or global spin limits.
Before requesting cancellation it atomically writes `guard-<uuid>.json` into
the configured `trajectory_log_dir` (default `rbnx-build/data/trajectories`).
The result callback reads only the matching UUID: safety stops become `FAILED`
with `safety_stop` and actual XY/yaw errors, while explicit user cancellation
remains `CANCELED`. Normal success still requires Nav2's real successful result.
This preserves the failure cause for Planner rather than masking an unfinished
goal; Planner may still fail if bounded recovery cannot complete navigation.
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

1. resolves the selected Src providers;
2. resolves and materializes the deployment-owned Nav2 YAML;
3. starts an optional PointCloud2-to-LaserScan adapter;
4. starts Nav2 and waits for the `navigate_to_pose` action server;
5. connects to Nav2's live `speed_limit` subscriber;
6. exposes navigate, status, cancel, and dynamic speed capabilities.

Missing required providers return `deferred`. Invalid config or a Nav2 startup
failure returns `error` and tears down every child process.

## Build and tests

Navigation generates only its Src MCP bindings on every deployment target.
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

### Pause and resume

When `provider_ids.odom` is `g1_chassis`, navigation also requires a healthy `g1_chassis` status (hardware ID `g1_chassis`) on `/diagnostics`, received within 0.5 s. Missing, stale or faulted diagnostics prevent new goals, queued goal dispatch, idle greeting authorization and resume, even if fresh measured odometry continues for display. A reported motion fault remains latched until the project is restarted. Update the chassis adapter and reload navigation together; an older adapter without these diagnostics cannot authorize G1 navigation.

MCP `navigate/pause` cancels the underlying action while keeping the public run ID PAUSED. Status reports `stationary=true` in detail after fresh measured motion remains below 0.02 m/s and 0.03 rad/s for 0.5 seconds. Resume requires an explicit current paused run ID and fresh stationary odometry, and queues the original destination. Executor already supports PAUSED. User cancellation prevents resume; new goals are rejected while pausing or paused. These MCP-only capabilities use package-local contracts and IDL; regenerate with `rbnx codegen -p . --mcp`. Run offline checks with `python3 -m unittest test_pause_resume test_guard_failure`.

With an empty run ID and no pending, running or paused public runs, `pause` still returns `accepted=false` (nothing was paused). Its detail is `no active navigation; stationary=true` only when Nav2 is ready, fresh odometry confirms sustained stopping, and the Nav2 action status subscription reports no active goal, including goals from RViz. The greeting skill uses this detail to allow idle greetings without a resume request. Other rejections do not authorize greeting. This check is not a reservation against later external goals; avoid sending manual motion or external goals during greetings.
