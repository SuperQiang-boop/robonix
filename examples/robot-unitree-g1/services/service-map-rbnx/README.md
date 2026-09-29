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

For Fast DDS on a host with multiple network interfaces, export
`RMW_IMPLEMENTATION=rmw_fastrtps_cpp` and `FASTRTPS_DEFAULT_PROFILES_FILE`
before boot. The Docker launcher mounts that readable XML file read-only and
forwards its container path. An invalid path fails before startup; leaving
the variable unset preserves the image's default profile.

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

## Maps, modes and session state

### Three databases, kept apart

RTAB-Map never writes a saved map. Knowing which file is which explains every
rule below.

| database | path | who writes it |
|---|---|---|
| saved map | `{MAPPING_MAPS_DIR}/{map_id}/rtabmap.db` | `save_map` only, once — immutable afterwards |
| runtime database | `{MAPPING_RUNTIME_DB_DIR}/…` (default `/tmp/robonix-mapping-runtime`) | RTAB-Map, continuously. A mapping session gets a fresh empty one; a load gets a copy of the saved map |
| legacy default | `~/.ros/rtabmap.db` | nothing, in a normal deployment — kept only as a last-resort fallback for `save_map` |

A saved map directory holds the artifacts alongside the database:

```
maps/lab_3f/rtabmap.db  occupancy.pgm  occupancy.yaml  occupancy.png  cloud.pcd  meta.yaml
```

`map_ops` keeps one record of which database is live and hands it to
`save_map`. Every entry point — deployment config, gRPC, MCP, the web UI —
updates the same record, so a map saved after a load snapshots the database
RTAB-Map actually holds open.

### Startup configuration

`map_mode` and `map_id` in the deployment config choose the startup state.
There are two useful combinations:

```yaml
# Build a new map. This is the default: omit both keys and you get it.
mapping:
  config: {}

# Come up localized on a saved map — the stable-frame form used for tasks.
mapping:
  config:
    map_mode: localization
    map_id: lab_3f
```

- `map_mode` defaults to `mapping`, which always opens a **fresh, empty**
  runtime database. A `map_id` given alongside it is ignored: it names a saved
  artifact, not a live session, and mapping never reopens one. There is no
  "start up and keep extending map X" configuration.
- `map_mode: localization` requires `map_id`, and fails to start if that map
  is missing — deliberately, rather than silently mapping from the boot pose.
  It copies the saved database to a runtime path and localizes against the
  copy, so the **map frame is stable across restarts** and Scene can load
  semantic state for the same id.
- `reset_map: true` is only meaningful in mapping mode.

### Runtime operations

| operation | changes the database | changes the mode | changes the frame |
|---|---|---|---|
| `save_map(map_id)` | no — snapshots the live database into a new saved map | no | no |
| `load_map(map_id)` | yes — copies the saved map and switches onto the copy | yes, to localization | yes, to the loaded map's frame |
| `switch_mode(mode)` | no | yes | no |
| `reset_map` | no — clears working memory, the file stays | back to mapping | **yes** — origin becomes the robot's current pose |
| `pose_estimate(x, y, θ)` | no | no | no |
| `delete_map(map_id)` | removes a saved map from disk | no | no |

Notes that matter in the field:

- **`save_map` publishes once.** Saving under an existing `map_id` is refused;
  a corrected map goes under a new id.
- **`load_map` always localizes.** A `mapping` mode argument is accepted and
  coerced, because RTAB-Map only restores the saved occupancy grid when the
  database is opened for localization.
- **`reset_map` invalidates coordinates.** The rebuilt map does not share the
  old frame, so positions recorded against the previous map are stale. The
  lifecycle broadcast bumps its generation to say so.
- **Loading replaces the live session.** Anything mapped since the last save
  is gone; save first.

### Switching mode at runtime

The config's `map_mode` is only the startup default. `switch_mode` flips the
running RTAB-Map without touching the database or the frame.

**Prefer a restart over a runtime switch.** Going from localization back to
mapping is the risky direction, and the loaded map usually leaves the live view
when you do it. The web UI shows a standing warning while localized and asks
for confirmation before that switch. To build a new map, restart the service
with `map_mode: mapping` instead.

#### Why the loaded map disappears

Nothing is deleted — the map becomes a graph component the published map is
not assembled from. Four steps, all in RTAB-Map 0.23.x:

1. Entering localization calls `Memory::incrementMapId()`, which opens a new
   session id and flushes short-term memory. Every load does this, because a
   load always enters localization.
2. While localized, each new node is dropped again rather than kept
   (`moveToTrash(_lastSignature, …)`), so the session id stays put.
3. Switching back to mapping only flips `Mem/IncrementalMemory` to true.
   `Memory::addSignatureToStm` links a new node to the previous one **only
   when their session ids match**, so the first node built after the switch
   gets no odometry link back into the loaded map. The graph now has two
   disconnected components.
4. The published map comes from `Rtabmap::optimizeCurrentMap`, which optimizes
   the connected component around the current node. The loaded map is in the
   other component, so it is not in `/map`.

It comes back when RTAB-Map detects a loop closure between the two sessions:
that link joins the components and the whole map returns. So the switch is
only safe where relocalization can actually succeed. The database on disk is
never affected either way.

### Workflows

1. **Build the first map** — start with no `map_id` / `map_mode`, drive the
   space, `save_map("lab_3f")`.
2. **Build another map** — restart the service, then drive and save under a new
   id. Do not load an existing map first.
3. **Run tasks on a saved map** — start with `map_mode: localization` and
   `map_id`, or `load_map(id)` on a running service.
4. **Correct a saved map** — build a fresh session and save under a new id; a
   published map is immutable.

## RTAB-Map UI

The RTAB-Map viewer starts with every mapping session so the graph, loop
closures and per-node grids are visible while the robot drives. It needs an X
server: `scripts/start.sh` forwards the host `DISPLAY` into the container, and
a session without one logs a line and continues headless. Set
`MAPPING_ENABLE_VIZ=false` to keep it off on a robot that has a display but no
operator.

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
  highlight shows the mode the service reports, so a mode changed by config,
  MCP, a load or a reset shows up here too. Localization ⇒ mapping raises a
  warning first (see *Switching mode at runtime*).
- **Reset map** — wipe the live SLAM session and rebuild from scratch (for
  when mapping diverges). Note: the origin resets to the robot's *current*
  pose, so the rebuilt frame won't match the old map (origin drift).
- **Set pose estimate** — arm the button, then press where the robot is and
  drag the way it faces. The heading matters as much as the position: seeding
  the right spot facing backwards fails to relocalize just as a wrong spot
  does. The **activity log** records the seeded pose and, a few seconds later,
  where it converged and how far that is from your estimate.
- **Live lidar overlay** — the current range returns are drawn on the map in
  green (2-D scan) and blue (point cloud). This is the check that answers "is
  localization right": if the returns do not sit on the walls of the map, the
  pose is wrong. Topics come from whatever Atlas resolved for
  `robonix/primitive/lidar/lidar` and `robonix/primitive/lidar/lidar3d`, so the
  overlay follows the deployment's capability bindings. When no 2-D scan
  capability is bound the page looks for a `LaserScan` on the graph instead and
  says which one it picked — a robot whose scan is projected downstream from a
  3-D cloud (and therefore never declared) still gets its overlay. Pin one with
  `webui_scan_topic` in the deployment config, or `MAPPING_WEBUI_SCAN_TOPIC`.
  This overlay follows the deployment's capability bindings — a Webots TIAGo shows
  its 2-D scan, a Ranger with a mid360 shows its cloud, and a deployment with
  no lidar bound simply has no overlay. Cloud returns are limited to a band
  around the sensor plane (`MAPPING_WEBUI_CLOUD_Z_BAND`, default 0.35 m) so
  they can be compared against a 2-D grid, and both are subsampled to
  `MAPPING_WEBUI_MAX_POINTS` (default 1200).

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
├── src/mapping_rbnx/webui.py             operator page server + ROS subscriptions
├── src/mapping_rbnx/webui_static/        the page itself (index.html, app.js, style.css)
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
- **Map origin drifts between runs** — you're in `mapping` mode (origin =
  boot pose). Use `localization` to re-anchor to the saved map.
- **`save_map` says "no live rtabmap database found to snapshot"** — nothing
  has opened a database yet, or the recorded one was removed. The message
  lists the paths it tried. A service that has been running and mapping always
  has one; if this appears right after a load, the deployment predates the
  shared live-database record and should be updated.
- **The map "disappeared" after switching to mapping mode** — expected, see
  *Why the loaded map disappears*. It returns on a loop closure with the loaded
  session; the saved map on disk is intact either way. Build new maps from a
  restart instead of a runtime switch.

License: MulanPSL-2.0
