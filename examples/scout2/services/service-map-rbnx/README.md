# mapping_rbnx

SLAM mapping service for [Robonix](https://github.com/syswonder/robonix). It
turns a robot's lidar / camera / odom streams into a live **2D occupancy
grid**, a **3D point cloud**, and a **SLAM-corrected pose**, published under a
fixed, engine-agnostic capability surface (`robonix/service/map/*`), and
persists named maps so a robot can re-localize across restarts.

It is a Robonix **service** package: it registers with `atlas`, discovers its
sensor inputs by capability contract (never hardcoded topics), and is brought
up by `rbnx boot`. Consumers (`scene`, `nav`) bind the contracts, not the
SLAM engine.

- Capability surface, config schema, and persistence layout: **[CAPABILITY.md](CAPABILITY.md)**.
- Provider instance configuration reference: **[config.spec](config.spec)**.

## Scout2 automatic relocalization

The Scout2 deployment can optionally use the independent
`examples/scout2/services/service-relocalization` package. Use one Scout2
manifest with `map_mode: auto`. Set `RELOCALIZATION_PACKAGE_ROOT` and
`RELOCALIZATION_COORDINATION_DIR` in its environment and the corresponding
config paths; relative paths resolve from the deployment directory. Docker
mounts the shared directory writable and the service package read-only.
Native deployments use the same host paths. Mapping records the saved database SHA-256 while making its
runtime copy and publishes private readiness only after a confirmed database
load and occupancy check. Existing capability contracts are unchanged.

Map load, reset and mode changes invalidate localization before acquiring an
exclusive lease. With navigation running, they wait for its velocity guard to
confirm stationary odometry. Failed coordination leaves localization invalid.
During mapping the relocalization service waits; periodic automatic saving
continues. Localization requests start only after a ready localization load,
controlled by `auto_relocalize` in the relocalization service config (default
true). With coordination enabled, switching from mapping to localization first
checkpoints and reloads the current named map through the checked load path.
See the relocalization README and Scout2 `scout2-start.md` for configuration.

Successful periodic mapping checkpoints and the final shutdown checkpoint also
save Scene's object snapshot for the active lifecycle `map_id` and generation.
This uses `POST /api/maps/save` with `spatial_saved: true`; Scene validates the
frame and writes its object partition and `scene_maps/<map_id>.json` without
calling mapping's save capability again. `SCENE_WEB_URL` defaults to
`http://127.0.0.1:50107`. A failed object save logs a warning while preserving
the spatial checkpoint; the next automatic checkpoint retries. SLAM remains
online throughout. Query saved objects with `GET /api/maps/<map_id>/objects`.

## SLAM engines (`algo`)

| algo | use | inputs |
|---|---|---|
| `rtabmap` *(default, recommended)* | sim + real; 2D/3D/RGBD, sensor-agnostic | any of lidar2d / lidar3d / rgbd (+ odom) |
| `dlio` | real-robot 3D Livox + IMU | lidar3d + imu, needs a colcon ws at `/ws/install` |
| `fastlio2` | **broken (drift)** — repro only | — |

The launch branches on the provider roles bound by the deployment, so the same
service supports 2D lidar, 3D lidar, RGB-D, and external odometry without
robot-specific branches.

## How to integrate it on your robot

1. **Register your sensors** as Robonix primitives under the standard
   contracts (`robonix/primitive/lidar/lidar3d`, `.../camera/depth`,
   `.../chassis/odom`, …). mapping discovers them via atlas.
2. **Pick a deployment target** and reference the matching package manifest
   from your deploy `robonix_manifest.yaml`:

   ```yaml
   service:
     - name: mapping
       url: https://github.com/syswonder/service-map-rbnx
       # manifest: package_manifest.jetson-native.yaml   # x86+docker is default
       config:
         sensor_providers:
           lidar3d: roof_lidar
           rgb: front_camera
           depth: front_camera
           odom: base_chassis
         occupancy_sources: [lidar]
         deskew_lidar: true
         params_file: config/rtabmap_params.yaml
         rtabmap_params:
           Grid/FootprintLength: 0.84
           Grid/FootprintWidth: 0.60
   ```
3. `rbnx build -f robonix_manifest.yaml` then `rbnx boot -f robonix_manifest.yaml`.
4. Consume the map: subscribe to `robonix/service/map/occupancy_grid` /
   `.../pointcloud` / `.../pose` (resolve them via atlas).

[`config/rtabmap_params.template.yaml`](config/rtabmap_params.template.yaml) is
only a starting template. Copy it into the robot deployment repository and set
`params_file`; Mapping never loads the upstream template at runtime. Inline
`rtabmap_params` applies after the deploy-owned file.

With external odometry, `deskew_lidar` compensates each PointCloud2 point in
the odom frame before SLAM. Bind only the sensor roles Mapping should consume.

### Separate localization and navigation odometry

Some robots need accurate ICP/RGB-D odometry for RTAB-Map localization but a
lower-latency chassis odometry stream for navigation. Enable the optional
split-odometry bridge for this setup:

```yaml
service:
  - name: mapping
    url: https://github.com/syswonder/service-map-rbnx
    config:
      algo: rtabmap
      base_frame: base_link

      # Private RTAB-Map odometry frame.
      odom_frame: odom_icp

      # Public chassis odometry used by navigation.
      navigation_odom_bridge: true
      navigation_odom_topic: /odom
      navigation_odom_frame: odom

      sensor_providers:
        lidar3d: roof_lidar
        # Do not bind odom in split-odometry mode.
```

In this mode, RTAB-Map uses an internal message-only odometry trajectory in
`odom_icp`, the chassis owns `odom -> base_link`, and Mapping publishes the
correction required by navigation:

```text
map -> odom -> base_link
```

The bridge computes `map -> odom` from RTAB-Map localization and the two
timestamp-aligned odometry poses. `odom_frame` and `navigation_odom_frame`
must be different, and `sensor_providers.odom` must not be configured.

The feature defaults to `false`; existing external- and internal-odometry
deployments are unchanged. See [config.spec](config.spec) for the complete
field definitions.

## Deployment targets

One package, three targets (selected by the deploy `manifest:` field — see
[CAPABILITY.md](CAPABILITY.md#deployment-targets)):

| target | manifest | runtime |
|---|---|---|
| x86_64 + docker | `package_manifest.yaml` | docker (`docker/Dockerfile`) |
| arm64 Jetson + docker | `package_manifest.jetson-docker.yaml` | docker (`docker/Dockerfile.jetson`, L4T) |
| arm64 Jetson + native | `package_manifest.jetson-native.yaml` | host ROS2 (`scripts/start_native.sh`) |

Add a target by adding a `package_manifest.<target>.yaml` plus a case branch
in `scripts/build.sh` — the rest of the package is unchanged.

The generated ROS 2 overlay intentionally builds only Robonix's custom `map`
interface package. Standard interfaces such as `sensor_msgs` continue to come
from the target's ROS 2 Humble installation, preserving its support libraries
and CMake exports for consumers such as `cv_bridge`.

## Saving & re-using a map

Named mapping sessions keep a writable runtime database under
`{MAPPING_RUNTIME_DB_DIR}/{map_id}/`, which must be on a Docker bind mount.
RTAB-Map writes nodes and links to its live database while mapping; automatic
optimized-map checkpoints publish a stable map under
`{MAPPING_MAPS_DIR}/{map_id}/` (default: the package's `maps/` dir):

```
maps/lab_3f/rtabmap.db  occupancy.pgm  occupancy.yaml  occupancy.png  cloud.pcd  meta.yaml
```

- **Build or continue a map:** set `map_id` plus `map_mode: mapping`. The first
  boot creates a runtime database; later boots restore the latest published
  checkpoint into the runtime database before continuing. While the robot
  drives, periodic checkpoints request the optimized graph and sensor data
  through RTAB-Map `/get_map_data`. They write a separate database without
  closing the engine or reading its live SQLite file. A final checkpoint is
  published at clean shutdown as well.
- **Re-use a map:** set `map_mode: localization`. Mapping copies the configured
  saved db to a private runtime path; if `map_id` is omitted it selects the
  newest saved map. This mode fails when no saved map exists. The saved graph
  is retained, but robot localization must be checked before using stored
  Scene coordinates or navigation goals.
- **Choose a map at startup:** set `map_mode: auto`. The newest saved map by
  directory modification time is selected (map ID breaks timestamp ties).
  The configured `map_id` names a new mapping session only when no saved map
  exists. Use explicit `localization` plus `map_id` to select a particular map.
- **Start fresh:** set `reset_map: true` with `map_mode: mapping`. This clears
  the named runtime database before rebuilding; without `reset_map`, mapping
  resumes the existing runtime database.

Named mapping restores the last published checkpoint and resumes the graph
with incremental mapping enabled. Restoring
the graph does not prove that the robot has relocalized: begin in an overlapping
known area and verify scan alignment / pose before moving into unexplored space.
If needed, localize with an initial pose first, then use `switch_mode(mapping)`;
this moves the loaded copy onto the persistent runtime path before extending it.
Graph optimization can still move stored map-frame coordinates.

### Scout2 Docker persistence

The Scout2 manifest uses `map_id: scout2_main`, `map_mode: auto`, and
`checkpoint_interval_s: 30`, with mapping and relocalization in the same
deployment. With no saved maps, mapping starts a new named map and the
relocalization service waits without preparing an index or issuing a request.
Once saved maps exist, `auto` selects the newest saved map for localization;
the manifest's `auto_relocalize` setting controls the startup request. Scene
uses the map lifecycle broadcast instead of a fixed map ID. Run `rbnx build`
and `rbnx boot` from the Scout2 deployment directory as usual. The package bind mount preserves:

- Host live DB: `services/service-map-rbnx/maps/.runtime/scout2_main/rtabmap.db`
- Host checkpoint: `services/service-map-rbnx/maps/scout2_main/rtabmap.db`
- Container equivalents: `/mapping/maps/.runtime/scout2_main/rtabmap.db` and
  `/mapping/maps/scout2_main/rtabmap.db`.

Every configured interval after the preceding save completes, the bridge runs
`rbnx-build/snapshot/mapping_snapshot`. This helper uses the existing
`/rtabmap/get_map_data` and `/rtabmap/get_parameters` services and RTAB-Map's
official database codec. It includes nodes still in memory, saves real optimized
poses, preserves sensor extrinsics, and assembles occupancy and cloud previews
from that same graph. It refuses disconnected graphs, missing valid nodes,
missing scans/local grids, and service failures; the previous checkpoint stays
available when saving fails. Invalid removed nodes (weight below -1) do not
count as disconnected components. `checkpoint.json` records the saved node
and scan-point counts. Normal shutdown also
publishes a checkpoint before stopping the engine. This is periodic
durability, **not per-frame zero-loss recording**: a crash can lose changes
since the latest successful snapshot (longer when saves fail). Setting the
interval to `0` disables periodic saves, not shutdown saves. Saving never pauses
or restarts SLAM. Exporting the graph uses the engine's processing callback;
its cost grows with map size, so monitor processing delays on the actual robot.
There is no direct online SQLite read or periodic `/backup` call.

`rbnx build` compiles the helper for the selected target (Docker or native),
using ROS Humble and RTAB-Map **0.23.7**. CMake and a C++ compiler are required;
Docker images include them. `MAPPING_SAVE_SNAPSHOT_TIMEOUT_S` defaults to `120`
seconds and bounds the whole helper process (plus 10 seconds for cleanup).
An absent helper is an explicit save failure requiring a rebuild.

`MAPPING_MAPS_DIR` changes the host/container map root; `MAPPING_RUNTIME_DB_DIR`
can set a separate bind-mounted runtime root. `ROBONIX_MAPPING_STOP_TIMEOUT_S`
defaults to `180` seconds; large graphs may need longer for a final save and
engine close. Avoid repeated forced termination. Keep the host maps directory
when rebuilding/recreating containers, and run only one writer per runtime DB.
An invalid runtime DB is preserved in a `previous-*` directory and restored
from the checkpoint; without a valid checkpoint startup fails instead of silently
creating an empty map. `rtabmap.previous.db` retains the previous published DB
for manual rollback. Setting `reset_map: true` starts a new map **on every boot**;
return it to `false` after an intentional reset.

Offline regression tests:
`PYTHONPATH=src /usr/bin/python3 -m unittest discover -p 'test_*.py'`.
The isolated ROS test `scripts/test_persistence_ros.py` requires the package at
`/mapping`, Scout2 at `/deploy` (read-only mounts), `PYTHONPATH=/mapping/src`, ROS
Humble sourced, and a Docker `--network none` container. It feeds scans during
multiple executions of the production automatic-save loop, checks graph growth
across normal/abrupt restarts, and rejects deliberately incomplete responses.
Set `MAPPING_TEST_EXPORTER` to the built `relocalization_export` executable and
include the relocalization package in `PYTHONPATH` to require formal export and
index preparation on every saved checkpoint. `MAPPING_PERSISTENCE_TEST_DIR`
retains the isolated test databases/logs in a new output directory. It must not
run on the robot's live ROS graph.

## Web UI (live map + runtime map ops)

A dependency-light operator page (stdlib `http.server` + Pillow) is enabled on
port `8091` by default; set deployment config `webui_port: 0` to disable it.
It binds `127.0.0.1` by default because the map controls are unauthenticated.
An authenticated overlay deployment may explicitly set `webui_host` (or
`MAPPING_WEBUI_HOST`); otherwise use the local browser or an SSH tunnel.

It runs **inside the mapping bridge process**, so its buttons call the same
`map_ops` impls the gRPC/MCP capabilities use — no extra round trip — and it
reads the live `/map` + pose straight off the bridge's rclpy node.

- **Live map canvas** — occupancy grid + robot pose, with **drag-to-pan,
  wheel-zoom, a 1 m grid, and double-click-to-fit**. Same world-centered
  view model as scene's web UI (canvas backing-store pinned to display size,
  so click coordinates are exact).
- **Save** — snapshot the live map under a `map_id` (writes
  `rtabmap.db` + `occupancy.png/pgm/yaml` + `meta.yaml`).
- **Library** — every saved map with a thumbnail; **Load** re-localizes onto
  it, **Del** removes it from disk.
- **Mode** — flip **Mapping ⇄ Localization** at runtime; a badge + button
  highlight shows the current mode.
- **Reset map** — wipe the live SLAM session and rebuild from scratch (for
  when mapping diverges). Note: the origin resets to the robot's *current*
  pose, so the rebuilt frame won't match the old map (origin drift).
- **Click the map → pose estimate** — seeds `/initialpose` so rtabmap
  re-localizes; the **activity log** records the seeded pose and, a few
  seconds later, where it converged + the distance from your estimate.

These are the same operations exposed as runtime **RPC + MCP capabilities**
(so Pilot can drive them too): `save_map`, `load_map`, `pose_estimate`,
`switch_mode` (the webui adds `reset` + `delete` on top). All work on the
running rtabmap without a redeploy — `load`/`switch_mode` call rtabmap's
runtime services and fall back to a restart with the config's `map_mode` /
`map_id` when those services aren't reachable.

> The web UI has no auth — it's a LAN debug tool. Don't expose the port to an
> untrusted network.

## Layout

```
mapping_rbnx/
├── package_manifest.yaml                 x86+docker (default)
├── package_manifest.jetson-docker.yaml   arm64 Jetson + docker
├── package_manifest.jetson-native.yaml   arm64 Jetson + host ROS2
├── CAPABILITY.md                         capability surface + config spec
├── src/mapping_rbnx/atlas_bridge.py      cap registration, sensor discovery, persistence
├── src/mapping_rbnx/map_to_odom_bridge.py optional split-odometry TF bridge
├── src/mapping_rbnx/odom_bridge_math.py   planar transform and interpolation helpers
├── launch/rtabmap_2d.launch.py           sensor-agnostic rtabmap launch
├── scripts/
│   ├── build.sh                          per-target build
│   ├── build_ros2_overlay.sh             isolated map interface build
│   ├── start.sh                          native↔docker dispatch
│   ├── start_engine.sh                   in-container SLAM launcher
│   ├── start_native.sh                   host-process launcher
│   └── save_map.py                       offline map snapshot (pgm/png/pcd/meta)
└── docker/                               Dockerfile, Dockerfile.jetson, compose
```

## Troubleshooting

- **`/map` never populates** — a provider binding is missing or points to the
  wrong provider. Check the `[start_engine] rtabmap scan2d=… scan3d=…` line.
- **`map_mode=localization` errors "no saved map"** — run a `mapping` session
  with that `map_id` first, and confirm `MAPPING_MAPS_DIR` is the same path
  (mounted) across runs.
- **Map alignment drifts between runs** — confirm a named DB was resumed and
  that current scans match the saved map. Localize/seed the initial pose before
  navigation; database recovery alone does not establish the robot's position.

License: MulanPSL-2.0
