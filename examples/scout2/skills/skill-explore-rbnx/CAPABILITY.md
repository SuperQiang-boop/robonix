---
description: Autonomous frontier exploration — drives the robot to map and catalog its surroundings (occupancy grid + semantic object registry).
---

# explore_rbnx — autonomous environment exploration skill

Drives the robot through frontier-based exploration of its surroundings,
building a spatial occupancy map (via the mapping service) and a
semantic object registry (via scene's built-in VLM detector) along the
way. User-invocable via the LLM/pilot.

## Interface (3 MCP tools)

### `robonix/skill/explore/explore`

Start an exploration task. Returns immediately with a `run_id`; poll
`status` to track progress. **If the user has not explicitly requested a
duration, pass `timeout_s=-1` to use the deployment default (currently no total
deadline). Do not invent a 300-second limit.** An explicit positive value
overrides the deployment; 0 also means no total deadline. Per-leg hard timeout,
stall detection, cancellation, and no-progress/no-safe-target termination
remain active. The start response message includes the effective timeout.

For Scout2, pass `max_speed_m_s=0.35` unless the user explicitly requests a
lower ceiling. This is a ceiling, not a constant driving speed; the navigation
controller slows for turns and obstacles. A lower requested ceiling requires
lowering the navigation session limit first.

| param            | type   | default | meaning                                    |
|------------------|--------|---------|--------------------------------------------|
| `area_hint`      | string | `""`    | free-form ("this room", "ground floor")    |
| `timeout_s`      | float  | -1      | -1 = deployment default; 0 = unlimited; positive = explicit override |
| `max_speed_m_s`  | float  | 0.35    | required positive ceiling for exploration; Scout2 Nav2 is configured for 0.35 m/s; does not bypass the velocity guard |

Returns `{accepted, run_id, message}`. `accepted=false` if a task is
already running or a previous navigation stop remains unconfirmed.

### `robonix/skill/explore/explore/status`

Poll an exploration's progress. Empty `run_id` returns the most
recent task.

Returns `{known, state, area_m2, frontiers_left, frontier_regions, safe_observation_candidates, budget_rejected_candidates, rejected_goal_clearance, rejected_route_reachability, rejected_visibility, rejected_view_distance, rejected_local_distance, rejected_near_robot, rejected_cooldown, viewpoint_cells_checked, current_target_valid, current_target_x, current_target_y, elapsed_s, eta_s, detail}`. Rejection counts classify each searched viewpoint cell by its first failed condition; later route filters apply only to candidates that passed viewpoint generation. `frontier_regions` is the latest-map cluster count; it is not a precomputed goal queue.
`state ∈ {PENDING | RUNNING | SUCCEEDED | FAILED | CANCELED | TIMEOUT}`
(terminal: SUCCEEDED / FAILED / CANCELED / TIMEOUT).

### `robonix/skill/explore/explore/cancel`

Abort the active exploration. Idempotent.

## Usage pattern (IMPORTANT — thread the run_id)

1. Call `explore` ONCE. It returns immediately with a `run_id`. **Save that
   exact `run_id`.**
2. To monitor, call `status` with that SAME `run_id` — repeatedly, until
   `state` is a terminal value (`SUCCEEDED | FAILED | CANCELED | TIMEOUT`). Do
   not call `explore` again to monitor; that starts nothing new (a task is
   already running) and loses your handle.
3. To stop it, call `cancel` with that SAME `run_id`.

Passing an empty `run_id` uses the most recent exploration task.
Before answering whether an exploration is still running, query `status` again
with its exact run_id. A RUNNING result from earlier in the conversation cannot
override a later FAILED/TIMEOUT result from the Executor or skill. `map/get_mode`
reports SLAM mode, not the exploration task's lifecycle. A no-candidate error
alone is not evidence of map corruption and does not justify a map reset.

## Behaviour

1. Wait for `/map` (mapping service publishes it).
2. Loop:
   - Find free frontier cells adjacent to unknown space; cluster and split
     large boundaries into local segments.
   - Search in two dimensions from actual boundary segments for known-free
     observation positions, within 2.5 m of the boundary. Each position needs a
     known-free sight line to its source boundary, a 0.65 m known-free clearance
     disk, and a clearance-safe route from the robot. Reject goals outside the
     15 m local range or in a recently failed-goal area; visits add a penalty.
   - Flood the clearance-safe known-free map from the robot pose; only score
     goals in the robot's reachable component. Nav2 still checks the actual
     path and full configured footprint.
   - Room traversal is based on the occupancy grid, not semantic door detection.
     Cross a doorway only if its known-free corridor belongs to the robot's
     0.65 m footprint-clear route component; never assume an unknown or narrow
     opening is traversable. Nav2 validates the actual route and footprint.
   - Read `navigation/get_speed_limit` and estimate known-free routes. Prefer
     shorter safe routes. The configured per-leg timeout is a hard cap; a route
     estimate above the advisory scheduling reference is still attempted when
     it fits that cap. Finite task deadlines reserve cancellation and arrival
     verification time. If a frontier route exceeds one hard-capped leg, choose
     a safe waypoint on the reconstructed known-free route and recompute after
     arrival; never send one over-cap route or interpolate across unknown space.
     Estimates do not replace Nav2's planner.
   - Send goal via `service/navigation/navigate` MCP; poll status.
   - Translation goals face the travel direction. A fresh robot pose held
     inside the configured 0.45 m observation radius for 2 seconds completes
     the exploration leg after navigation cancellation is confirmed. This is
     an exploration observation criterion; Nav2's own checker is unchanged.
   - In-place sweeps are disabled by default (`sweep_every_n_legs=0`). Opt-in
     sweeps require time and missing viewing coverage; sweeps without meaningful
     free-area growth are not repeated.
3. Declare `done` only when no frontier clusters remain and confirmed map
   growth/translation has been quiet for 30 seconds. Remaining frontiers with
   no safe reachable goal are reported as an error, not successful coverage.
   When no candidate is available, wait for at least a 30-second no-progress
   window from the first missing candidate or latest progress, not just five
   fast polls. Reports include start clearance, reachable cell count, generated
   viewpoints and distance/cooldown/budget rejection counts. A disabled sweep
   is reported as disabled, never as a performed rotation. Terminal elapsed_s
   is frozen at completion.

Progress is measured from accumulated free-area growth or at least 0.25 m of
translation since the last confirmed progress. Slow growth accumulates against
the last progress snapshot; in-place sweeps and RPC success alone do not reset
the 120-second no-progress limit. Failed goal regions are excluded for 120 s.
A finite task may report TIMEOUT before dispatch if its remaining time cannot
fit any reachable frontier. This does not mean the map is fully explored.
Cancellation is bound to the exact owned navigation run; an unconfirmed stop
blocks the next exploration. Navigation speed is queried, not raised; if the
current navigation limit exceeds `max_speed_m_s`, the task fails with an explicit
instruction to lower the navigation limit.

## What this skill does NOT do

- No SLAM (mapping service does that — atlas-resolved input).
- No collision avoidance (navigation service does that — atlas-resolved).
- No object detection (scene service does that, in-process VLM, on
  the camera frames published by the camera primitive).

The skill is purely a goal-selection loop with safety pre-filters. All
sensor/actuator access is through atlas-resolved contracts; no
hardcoded inter-package topic names.

## Dependencies it Connect()s on atlas at startup

| key             | contract                                | transport |
|-----------------|------------------------------------------|-----------|
| map_topic       | robonix/service/map/occupancy_grid       | ROS2      |
| nav_navigate    | robonix/service/navigation/navigate      | MCP       |
| nav_status      | robonix/service/navigation/navigate/status | MCP     |
| nav_cancel      | robonix/service/navigation/navigate/cancel | MCP     |
| nav_get_speed   | robonix/service/navigation/get_speed_limit | MCP     |

Skill refuses to start if any of these is missing — there is no
hardcoded fallback (packaging-spec invariant #1).

## Portability

The skill code is identical between webots and the real robot. What
differs is which packages back the consumed contracts:

- webots: mapping=rtabmap (2D lidar + RGBD), nav=simple_nav, chassis=tiago_chassis
- jetson AGX + Mid360: mapping=DLIO (3D livox + IMU), nav=nav2-wrapper, chassis=ranger_chassis

Both back the same `service/map/occupancy_grid` + `service/navigation/*`
contracts; explore neither knows nor cares which is running.
