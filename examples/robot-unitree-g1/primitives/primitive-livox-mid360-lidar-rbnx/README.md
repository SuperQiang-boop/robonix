# G1 MID-360 integration

This local primitive uses the point-cloud and IMU extrinsic rotation implementation
from `tmp/FAST_LIO_LOCALIZATION_HUMANOID/livox_ros_driver2/src/comm/pub_handler.{cpp,h}`.
Robonix launch environment support, ROS 2 build fixes and `/scanner/cloud` remain
from the original wrapper. IMU matrices are initialized from configuration before
samples arrive and stored per sensor under a mutex (instead of the source fork's
shared matrix initialized by the first point packet).

`src/livox_ros_driver2/config/MID360_config.json` configures
`lidar_configs[0].extrinsic_parameter`: angles are degrees, translation is zero.
G1 uses roll=180, pitch=yaw=0. Points, angular velocity and acceleration are rotated;
IMU units and timestamps remain as provided by Livox. The wrapper generates host
and lidar IP settings at initialization while preserving these extrinsics.

The deployment publishes corrected points in `mid360_upright`, and corrected IMU
in `livox_frame`. URDF retains the physical inverted `mid360_link`/`mid360_imu`
frames and adds a 180-degree rotation to each corrected output frame. The current
URDF leaves residual roll=-0.00248080 and pitch=0 radians after driver rotation.
Both outputs inherit this nominal mount when transformed to the robot frame;
the raw corrected sensor messages still express their own tilted axes.
Changing the driver rotation requires updating these output-frame transforms too.
The separate IMU primitive only registers `/livox/imu`; it needs no driver changes.

Runtime uses `/usr/bin/python3` and the ROS distribution's standard message
packages. `scripts/ros_env.sh` removes this primitive's generated ROS IDL overlay
from inherited paths to prevent Conda Python 3.13 messages shadowing Humble's
Python 3.10 type support. Build pins CMake's Python interpreter to `/usr/bin/python3`
and does not compile generated copies of standard ROS messages. Protobuf codegen
remains separate. Validate with `bash tests/ros_types_smoke.sh`, including from a
shell which previously sourced the generated IDL overlay.

Build with `bash scripts/build.sh` from this directory, then restart the deployment
to load the new driver and URDF. Do not run a second Livox driver against the same
sensor. Hardware validation should check both message frame IDs and that a level,
stationary sensor acceleration transformed into the robot frame points along +Z.

Validation: `/usr/bin/python3 -m unittest discover -s tests -v` and
`bash tests/extrinsics_smoke.sh`. The latter feeds synthetic IMU and point packets
to the actual driver code, checking rotation before the first point packet and
independent rotations for two sensor handles.

## Optional gravity alignment (G1)

The G1 deployment enables `gravity_alignment` in this lidar primitive. Raw
`/scanner/cloud` and `/livox/imu`, the URDF and existing TF edges stay available.
Atlas advertises `/scanner/cloud_aligned` through the existing lidar3d capability,
so RTAB-Map and Nav2 receive the same correction without consumer changes.
The aligned cloud retains every original point byte, field and timestamp but uses
`mid360_gravity_aligned`. Its new dynamic TF from `base_footprint` rotates about
the nominal sensor origin using timestamp-matched IMU gravity. Use this TF with
the cloud; inspecting XYZ bytes alone does not show the compensation.

Requires system Python `numpy`, `scipy`, `rclpy`, `sensor_msgs`, `geometry_msgs`
and `tf2_ros` (available in this G1 ROS environment). Start robot_description
before the lidar primitive so the mount TF exists, and keep the robot still for
at least one second at initialization. The IMU primitive can start later because
the Livox driver itself already publishes its IMU. Gyro propagation follows fast
tilt; a two-second accelerometer correction rejects magnitude errors over 8% and
direction jumps over 5 degrees. Initial gyro bias is measured while stationary.
Each cloud is rotated by the attitude at the midpoint of its per-point
`timestamp` span (Livox absolute nanoseconds; ~0.1 s sweep), not at the header
stamp, which is the first point; on the walking G1 this cut the median floor
tilt error from about 1.0 to 0.35 degrees. Clouds without a usable per-point
timestamp fall back to the header stamp with a throttled warning. Header and TF
stamps are unchanged.
This does not perform per-point deskew, estimate height, or identify arbitrary
sustained linear acceleration; walking still requires hardware validation.

IMU gaps over 0.1 s force stationary reinitialization. Clouds wait at most 0.25 s
for bracketing IMU and TF; missing data is logged and dropped, with no raw fallback.
Atlas is registered only after the first aligned cloud. The alignment startup
wait is bounded to ten seconds (or a shorter raw sentinel timeout). Roll/pitch
offsets in the G1 manifest are the empirical four-heading calibration from
2026-11-09, applied
to IMU vectors in nominal base axes (extrinsic X then Y), not physical mount angles.
Set `gravity_alignment.enabled: false` and restart the deployment to restore raw
cloud discovery. No C++ rebuild is required. Existing maps are not rewritten.

Run `/usr/bin/python3 -m unittest discover -s tests -v`. For an isolated ROS
process test, source `/opt/ros/humble/setup.bash`, unset
`FASTRTPS_DEFAULT_PROFILES_FILE`, and run:

```bash
ROS_DOMAIN_ID=217 ROS_LOCALHOST_ONLY=1 RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  /usr/bin/python3 tests/gravity_alignment_smoke.py
```

The smoke test publishes synthetic data only in that separate domain; it checks
TF compensation, metadata preservation, real low obstacles, IMU loss and recovery.

---

# mid360_lidar_rbnx

Robonix package wrapping the **Livox MID-360** LiDAR (Ethernet, 360° dome, 40 m range, integrated 6-axis IMU). Owns the `primitive/lidar/*` namespace. Publishes the lidar PointCloud2 on the host DDS bus and atlas-registers it under generic contracts so that mapping, navigation, and scene services discover the topic name through atlas — no hardcoded `/scanner/cloud` paths on the consumer side.

The MID-360 also produces an IMU stream (`/livox/imu`, published as a side-effect of the same upstream launch). The `primitive/imu/*` contract surface for that IMU lives in a separate package, [`mid360_imu_rbnx`](https://github.com/syswonder/primitive-livox-mid360-imu-rbnx). Robonix's invariant is "one primitive namespace = one package".

## Capability surface

The `mode` is the abstract communication pattern declared in the contract TOML (`rpc` / `topic_in` / `topic_out`). The `transport` column records how this package realises it on the wire. Both columns matter — the same mode can ride different middleware (e.g. an `rpc` mode can be a gRPC method or an MCP tool).

| Contract                                 | Mode      | Transport | Source / handler                            |
| ---------------------------------------- | --------- | --------- | ------------------------------------------- |
| `robonix/lifecycle/driver`               | rpc       | gRPC      | shared `Driver(CMD_INIT, config_json)` lifecycle |
| `robonix/primitive/lidar/lidar3d`        | topic_out | ROS 2     | `/scanner/cloud` (PointCloud2)              |
| `robonix/primitive/lidar/lidar_snapshot` | rpc       | MCP       | one-shot capture (TODO)                     |

## Driver-init lifecycle

`start.sh` brings up the atlas bridge process — no ROS spawn at this point. The shared Robonix runtime registers the lifecycle driver, then the provider blocks on heartbeat awaiting `Driver(CMD_INIT, config_json)`.

When `rbnx boot` invokes Init it passes the manifest's `config:` block as JSON. The handler resolves the host's IP for `Livox/host_net_info` (config override `host_ip:`, env `LIVOX_HOST_IP`, or auto-detect via `ip route get <lidar_ip>`), generates an MID360 config JSON with the right IPs, spawns `ros2 launch livox_ros_driver2 msg_MID360_launch.py` with the appropriate environment (`LIVOX_MID360_CONFIG`, `LIVOX_XFER_FORMAT`), waits for the first PointCloud2 on the configured topic, declares `primitive/lidar/lidar3d` on atlas, and returns ok. Atlas only ever advertises endpoints we've confirmed are publishing.

## Layout

```
mid360_lidar_rbnx/
├── package_manifest.yaml         robonix dev-packaging spec
├── mid360_driver/
│   └── atlas_bridge.py           driver gRPC + lazy Init + livox spawn
├── scripts/
│   ├── build.sh                  colcon build vendored src + rbnx codegen
│   └── start.sh                  source ROS, exec atlas_bridge
├── src/
│   ├── livox_ros_driver2/        VENDORED upstream + our patches
│   └── livox_ros_driver2.patch   diff vs upstream HEAD at vendoring time
└── .gitignore                    excludes rbnx-build/
```

## What we patched on top of upstream

`src/livox_ros_driver2.patch` documents the diff against [Livox-SDK/livox_ros_driver2](https://github.com/Livox-SDK/livox_ros_driver2). The vendored copy already has them applied:

1. `config/MID360_config.json` — host_net_info IPs `192.168.1.5 → .50`, lidar IP `.12 → .161`. Atlas_bridge can override at runtime via the `host_ip` / `lidar_ip` config keys; the JSON file just provides the baseline.
2. `launch_ROS2/msg_MID360_launch.py` — `xfer_format` default `1 → 0` (ROS2 `sensor_msgs/PointCloud2` instead of Livox CustomMsg). It also reads `LIVOX_XFER_FORMAT` / `LIVOX_PUBLISH_FREQ` / `LIVOX_FRAME_ID` from the env so the provider can pass config through. Format `2` is not valid on this ROS2 driver path because it attempts to publish `pcl::PointCloud`.
3. `src/lddc.cpp` — global publisher topic `livox/lidar → scanner/cloud`. `multi_topic=1` keeps publishing per-lidar topics under `livox/lidar_*` unchanged.

## Config (passed via `Driver(CMD_INIT, config_json)`)

```json
{
  "lidar_topic": "/scanner/cloud",
  "lidar_ip": "192.168.1.161",
  "host_ip": "",
  "xfer_format": 0,
  "publish_freq": 10.0,
  "frame_id": "livox_frame",
  "sentinel_timeout_s": 30.0
}
```

`host_ip: ""` triggers auto-detect via `ip route get <lidar_ip>`. Override only when route resolution can't pick the right interface (e.g. the Jetson has multiple Ethernet ports and the route picks the wrong one).

## Build / run standalone

```bash
bash scripts/build.sh           # or:  rbnx build -p .
bash scripts/start.sh           # or:  rbnx boot  -p .
# the driver gRPC will sit awaiting Driver(CMD_INIT). For a smoke test
# without rbnx boot, drive it manually with grpcurl or a local Python script.
```

After Init the lidar should appear on:

```bash
ros2 topic hz /scanner/cloud   # ~10 Hz PointCloud2
ros2 topic hz /livox/imu       # ~200 Hz sensor_msgs/Imu (consumed by mid360_imu_rbnx)
```

## Network — host-side prereqs

The lidar's IP is config-driven; the host's IP is not. Whatever the lidar's configured IP is (we ship `192.168.1.161` as default; flash to a different value via Livox Viewer 2 if your network policy dictates), the host's NIC must live on the same /24 subnet (e.g. `192.168.1.50/24` for a `192.168.1.0/24` lidar) and have a route to the lidar IP. How you set that up is host-specific — `nmcli`, `systemd-networkd`, plain `ip addr`, whatever your distro standardises on. The package can't subsume that step because Ethernet IP config is inherently host-state.

## License

This package: MulanPSL-2.0. Vendored `livox_ros_driver2/`: see `src/livox_ros_driver2/LICENSE` (BSD).
