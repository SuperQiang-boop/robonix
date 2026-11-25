# explore_rbnx

Autonomous frontier exploration skill for robonix. Driven by the
LLM/pilot via `rbnx ask "explore"` (or any equivalent intent that
triggers the `robonix/skill/explore/explore` tool).

See `CAPABILITY.md` for the full interface + behaviour spec.

Frontier goals are sampled by a bounded two-dimensional search from real
boundary cells into known-free space (up to 2.5 m observation distance). Goals
must have a known-free sight line to their source boundary; the observation
search is not a route the robot follows. A candidate must have a 0.65 m known-free clearance disk and belong to
the robot's clearance-safe connected region before Nav2 plans the route. Large
frontiers are split into local segments; recent failed-goal regions are cooled
down for 120 seconds. Exploration progress is cumulative free-area growth or
confirmed translation, not task age, navigation RPC completion, or in-place
rotation. Remaining frontiers without a safe reachable goal are not reported as
successful coverage.

No-target retries wait for an actual 30-second no-progress window rather than
failing after five fast polls of a static map. New measured map growth or
translation restarts this window. Selection diagnostics report robot clearance,
reachable cells, generated viewpoints and distance/cooldown/budget rejections.
Sweep-disabled runs do not claim to be rotating. Status elapsed time stops at
termination; query the exact run again instead of reusing an old RUNNING result.
These limitations are not evidence that the map needs resetting. Status also exposes frontier region count, safe observation candidates, budget-rejected candidates, and the current observation target.

## Quickstart

```bash
# from the package root
bash scripts/build.sh   # rbnx codegen + docker build
# then add to your deploy manifest under skill: and rbnx boot
```

Manifest snippet:

```yaml
skill:
  - name: explore
    path: ../../../explore_rbnx
    config: {}
```

The skill registers 3 MCP tools with atlas. Pilot's tool catalog
will include `robonix/skill/explore/{explore, status, cancel}`.

## Time limits and navigation

Deployment `config.timeout_s` is read and validated at INIT (default `0`, no
total task deadline). Explore requests use `timeout_s=-1` for the deployment
default, `0` for no total deadline, or a positive user-requested override. The
180-second per-leg hard cap, navigation cancellation, stalled-motion detection,
and no-progress/no-safe-target termination still apply. Unlimited total time
does not mean an individual navigation may run forever. Pilot uses `-1` when
the user did not specify a duration; an explicit duration remains a hard task
deadline.

The skill resolves `navigation/get_speed_limit` through Atlas and uses the live
metric limit, not the requested speed as an optimistic travel estimate. The
existing navigation limit must be positive and no greater than the requested
exploration cap. The skill does not raise or mutate the navigation session's
speed settings. Scout2's deployment configuration sets the navigation ceiling
and default session limit to 0.35 m/s; the live value is still checked before
each exploration loop.

Known-free route lengths are estimated using four-connected grid distances.
The estimate adds a 30-second turn allowance, a 1.5 travel-time factor, and
arrival-settling allowance; it is a reference, not a prediction of exact Nav2
completion. Safe goals are ranked by route length. Estimates above the advisory
reference are still tried when within the configured 180-second per-leg hard
cap. Unlimited tasks use the full cap; finite tasks reserve cancellation and
arrival verification from the global deadline, not inter-leg sleep/selection
overhead from each navigation. When no frontier goal fits one hard-capped leg,
the skill selects a safe clearance-qualified waypoint along a reconstructed
known-free route, stops there, and recomputes frontiers from the updated map.
If no useful waypoint fits, it reports the hard-limit/clearance reason rather
than issuing an overlong goal. Estimates remain heuristics; Nav2 validates paths.

When frontier regions remain but no observation target is selected, the status
reports first-failure counts for endpoint clearance, safe-route reachability,
known-free visibility and maximum frontier-view distance. It separately counts
later local-range, too-near, failed-goal cooldown and hard-budget rejections.
`viewpoint_cells_checked` reports how many boundary-view cell evaluations the
bounded search performed, so zero valid viewpoints can be diagnosed without
guessing.

Translation goals face the direction of travel. Fresh pose observations held
inside `arrival_tolerance_m` (default 0.45 m) for `arrival_settle_s` (2 s) end an
exploration observation leg after confirmed cancellation. Candidate endpoints
reserve the 0.65 m base clearance plus the configured arrival tolerance (default
endpoint radius 1.10 m). Nav2 then uses its unchanged 0.40 m XY / 0.25 rad yaw
goal tolerances and full configured footprint. The actual settled pose is checked
again at 0.65 m; an unsafe arrival cools down that goal and selects another
instead of counting it as success. Both Nav2 success and early XY completion
require this verification against the latest received map, using TF poses no
older than one second and stable XY for the settle interval. An unsafe current
pose ends the task with an explicit clearance error if no safe retry is possible.
The navigation service's exact goal checker and tolerances are unchanged. No
observed translation for 20 seconds after the initial turn allowance cancels a stalled
leg so another candidate can be tried.

`config.sweep_every_n_legs=0` disables autonomous in-place sweeps, appropriate
for this Scout2 Mid360 mapping deployment. Set a positive integer to opt in;
sweeps then require enough remaining time, missing angular coverage, and
measurable map benefit. See `config.spec` for all accepted config fields.

Room and doorway handling is occupancy-grid based, not semantic door
recognition. Frontier goals in other rooms are reachable when the mapped
known-free route passes the all-yaw 0.65 m chassis-clearance test; Nav2 then
checks the actual path and footprint. A narrow or still-unknown doorway is not
assumed traversable, so exploration can stop with frontiers remaining rather
than force the chassis through it.

## Verification

```bash
PYTHONPATH=. python3 -m unittest discover -s tests -v
```

The actual MCP client can also be tested with a synthetic local provider in a
network-isolated container. This test has no ROS endpoints or chassis access:

```bash
docker run --rm --network none \
  -e ROBONIX_EXPLORE_TEST_ISOLATED=1 -e PYTHONPATH=/explore \
  -v "$PWD:/explore:ro" --entrypoint python3 robonix-explore \
  /explore/tests/integration_navigation_mcp.py
```

## Layout

```
explore_rbnx/
├── package_manifest.yaml    # 3 caps + build/start hooks
├── capabilities/            # package-local TOMLs + .srv files
│   ├── explore.v1.toml
│   ├── status.v1.toml
│   ├── cancel.v1.toml
│   └── srv/{Explore,GetExploreStatus,CancelExplore}.srv
├── explore_skill/           # python module
│   ├── atlas_bridge.py      # entrypoint: register + serve MCP
│   ├── frontier.py          # WFD + clustering + scoring + safety
│   └── controller.py        # state machine, frontier loop, sweep
├── scripts/{build,start}.sh
├── docker/{Dockerfile, entrypoint.sh, no_shm_profile.xml}
├── CAPABILITY.md            # LLM-readable spec (registered with atlas)
└── README.md
```

License: MulanPSL-2.0
