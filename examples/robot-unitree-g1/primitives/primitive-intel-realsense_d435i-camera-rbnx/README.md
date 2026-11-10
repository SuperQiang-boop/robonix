# realsense_camera_rbnx

Robonix package wrapping the **Intel RealSense D435i** RGBD camera. Owns the `primitive/camera/*` namespace. Exposes the camera's RGB + aligned-depth streams under generic contracts so that mapping, scene, and any vision skill can resolve topic names through src (no hardcoded `/camera_435i/...` paths on the consumer side).

## Capability surface

| Contract                                  | Mode      | Transport | Source / handler                                          |
| ----------------------------------------- | --------- | --------- | --------------------------------------------------------- |
| `robonix/primitive/camera/driver`         | rpc       | gRPC      | `Driver(CMD_INIT, config_json)` — lifecycle gate          |
| `robonix/primitive/camera/rgb`            | topic_out | ROS 2     | `/<cam>/color/image_raw` (sensor_msgs/Image)              |
| `robonix/primitive/camera/depth`          | topic_out | ROS 2     | `/<cam>/aligned_depth_to_color/image_raw`                 |
| `robonix/primitive/camera/extrinsics`     | topic_out | ROS 2     | latched `geometry_msgs/TransformStamped` from the robot TF tree |
| `robonix/primitive/camera/snapshot`       | rpc       | MCP       | one-shot RGB capture (TODO)                               |
| `robonix/primitive/camera/depth_snapshot` | rpc       | MCP       | one-shot depth capture (TODO)                             |

The D435i's internal IMU is **deliberately not** registered under `primitive/imu/imu` — the Ranger Mini's MID-360 IMU is canonical for that contract (better noise, co-located with the lidar for SLAM). The camera IMU is still published to `/<cam>/imu` for anyone who wants it directly.

## Driver-init lifecycle

`start.sh` only brings up the src bridge process. The bridge opens a gRPC server on port `REALSENSE_DRIVER_PORT` (default 50232), registers the capability and declares only `primitive/camera/driver` on src, then blocks on heartbeat awaiting `Driver(CMD_INIT, config_json)`.

When `rbnx boot` invokes Init it passes the manifest's `config:` block as JSON. The handler parses config (camera name, RGB/depth profiles, align_depth, IMU on/off), spawns `ros2 launch realsense2_camera rs_launch.py …`, waits for the first frame on the configured RGB topic, declares `primitive/camera/{rgb, depth}` on src, and returns ok. Src only ever advertises endpoints we've confirmed are publishing.

## Layout

```
realsense_camera_rbnx/
├── package_manifest.yaml
├── realsense_camera/
│   └── src_bridge.py           driver gRPC + lazy Init
├── scripts/
│   ├── build.sh                  colcon build vendored src + rbnx codegen
│   └── start.sh                  source ROS, exec src_bridge
├── src/
│   └── realsense-ros/            VENDORED IntelRealSense/realsense-ros
└── .gitignore
```

## Config (passed via `Driver(CMD_INIT, config_json)`)

```json
{
  "camera_name":        "camera_435i",
  "rgb_profile":        "640x480x30",
  "depth_profile":      "848x480x30",
  "align_depth":        true,
  "enable_imu":         true,
  "enable_sync":        true,
  "sentinel_timeout_s": 30.0
}
```

`rgb_topic` / `depth_topic` can be overridden directly if your launch uses a non-default namespace.

The extrinsics publisher looks up `extrinsics_base_frame` (default `base_link`)
to `extrinsics_camera_frame` (default `camera_color_optical_frame`) in TF.
It retries every 0.5 seconds until successful or shutdown, logging the specific
TF error immediately and then every ten seconds while unavailable. RGB, depth,
and snapshots continue independently. Once available, the transform is published
once on `extrinsics_topic` (default `/camera/extrinsics`) with latched QoS for
late subscribers. This assumes the camera is fixed relative to the robot; it
does not create or calibrate the underlying TF transform.

## Build / run standalone

Build, codegen, and runtime use `/usr/bin/python3` so the Python ABI matches
the system ROS installation, even when Conda is active. Both CMake Python
interpreter variables are pinned during colcon builds. When switching from a
Conda-built workspace, move its generated `build`, `install`, and `log`
directories aside before rebuilding; changing the shell interpreter alone
does not replace old ROS overlays. Keep `src` and `rbnx-build/data` intact.
The vendored camera workspace uses copy installation with
`AMENT_CMAKE_SYMLINK_INSTALL=OFF`, including incremental rebuilds. Mixing a
previous copy build with `--symlink-install` makes ament fail with
`existing path cannot be removed: Is a directory`; the build script explicitly
resets this CMake option so the existing copy-built workspace remains usable.

```bash
bash scripts/build.sh
bash scripts/start.sh        # registers driver iface, waits for INIT
```

To drive Init manually (without rbnx boot): from any robonix gRPC client, call `PrimitiveCameraDriver.Driver` with `command=0` and `config_json='{}'` against `127.0.0.1:50232`.

## Tests

Run the hardware-independent retry smoke tests from this package directory:
`/usr/bin/python3 -m unittest discover -s tests -v`. They cover TF recovery after the old
60-second deadline, error throttling, and cancellation on shutdown.
After building, `test_ros_overlay.py` also imports actual native ROS type
support through the same overlays as `start.sh`, without initializing a ROS
node or starting a camera. Importing `Parameter` alone does not load its native
type support and cannot detect a Python ABI mismatch.

## License

This package: Apache-2.0 (matches realsense-ros upstream).
