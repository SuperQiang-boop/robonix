# Unitree G1 Deployment

Unitree G1 bipedal humanoid robot deployment for Robonix semantic navigation.

## Hardware

| Component | Model | Location |
|---|---|---|
| LiDAR | Livox Mid-360 3D | Head |
| RGB-D Camera | Intel RealSense D435i | Head |
| Locomotion Controller | Unitree SDK2 LocoClient | — |
| Arms | Dual arms (Dex3 hand option) | Torso |

## Getting Started

```bash
# 1. Initialize SDK2 submodule
git submodule update --init --recursive third_party/unitree_sdk2

# 2. Build
bash build.sh

# 3. Boot (no-motion for first-time)
bash start.sh --no-motion

# 4. Boot with motion (after operator confirms)
bash start.sh
```

## RViz lidar display range

To show only points within 6 m of the lidar, run in the same ROS environment
as the deployment (ROS Humble, `numpy`, `rclpy`, and `sensor_msgs` required):

```bash
source /opt/ros/humble/setup.bash
python3 scripts/g1_rviz_cloud.py --max-range-m 6
```

In RViz, set the PointCloud2 display topic to `/scanner/cloud_rviz`. The filter
uses 3D distance from the lidar and preserves point fields, frame and timestamp.
Keep the script running while viewing; stop it with Ctrl-C. Raw `/scanner/cloud`,
mapping and navigation ranges remain unchanged. Offline check:
`python3 -B -m unittest discover -s tests -p test_g1_rviz_cloud.py`.

## USB microphone on G1, Robonix on the laptop

G1 needs only `alsa-utils`, Python 3.8+ and `websockets>=12`; it does not need
Robonix. Copy `scripts/g1_usb_audio_server.py` to G1. On G1, verify card 2's
capture device with `arecord -l`, install the dependencies if missing, then run:

```bash
# On G1; device 0 is assumed. Use an existing Python environment if preferred.
python3 -m venv /tmp/g1-audio-venv
/tmp/g1-audio-venv/bin/python -m pip install 'websockets>=12,<14'
/tmp/g1-audio-venv/bin/python /tmp/g1_usb_audio_server.py \
  --device plughw:2,0 --host 192.168.123.164 --port 60000
```

Keep this process running before starting Robonix on the laptop. It verifies
one second of ALSA capture at startup and records on demand thereafter. Each
WebSocket frame is 100 ms of raw 16 kHz mono s16le PCM. Disconnecting stops and
reaps `arecord`. A new capture waits up to three seconds for the previous
capture to release the microphone; if it remains busy, the new connection
is rejected without interrupting the existing recording. Errors remain in the G1
terminal. Use the wired G1 address in both this command and the manifest's
`g1_usb_audio.config.host`; the current address follows the camera deployment.
The endpoint has no authentication; bind it to the trusted wired interface.
The temporary venv and script must be recreated after G1 reboots.

The laptop runs two instances of the official audio client bridge:
`g1_usb_audio` connects to G1's capture server, while `audio_client_bridge`
retains browser/client audio on port 60002. Both use the existing audio
contracts; the official ALSA primitive alone cannot read another host's USB.
No cached upstream source is modified. Build and boot from the laptop as above.

The manifest sets `ROBONIX_CLIENT_SETTINGS` to the local frontend's
`/home/ygq/.config/robonix-client/settings.yaml`. `g1_face_greeting` reads the
saved Audio input from this file before every capture, with
`speaker_provider_id: g1_audio` and `voice_enabled: true`. Select the microphone
provider and input device in Audio, then choose Apply Route. The next greeting
uses that input without a skill restart. Keep the client's audio device server
running for laptop input. To capture from G1 again, first verify its recording,
then select `g1_usb_audio` in Audio. Missing or invalid shared settings prevent
capture instead of selecting a different microphone. This file sharing assumes
the frontend and skill run on the same host; if the frontend uses a custom
`ROBONIX_CLIENT_SETTINGS` path, set the deployment to that same absolute path.
Without this environment variable, the skill uses `mic_provider_id`.
Liaison records after playback waiting and confirmed arm release, uses
`speech/asr_stream` (FunASR in local mode), then submits the request to Pilot.
The G1 USB server offers input only. Do not select it as a speaker.
Apply Route can confirm the current input or its default without restarting
capture. To use a different ALSA device, restart the G1 script with `--device`.

The official bridge's legacy mode logs WebSocket failures but ends the gRPC
mic stream normally. Liaison may interpret a failed capture as silence or use
partial audio. The G1 capture server allows a bounded wait while the bridge
releases its previous capture, including wake-word to command transitions.
The tests below verify repeated gRPC capture handoffs, PCM transport and
G1-side cleanup, not
failure propagation into voice task admission. Keep automatic greeting input
from G1 disabled pending hardware validation and upstream error handling improvements.

Laptop transport checks (synthetic PCM, no robot motion or physical microphone):

```bash
rbnx-boot/cache/primitive-audio-client-bridge-rbnx/rbnx-build/venv/bin/python \
  -B -m unittest discover -s tests -p test_g1_usb_audio.py
```

`speech/speak` keeps `default_speaker_provider_id: g1_audio` and
`UnitreeTTSBackend`: SDK2 `TtsMaker` performs synthesis and playback on G1.
This route sends text, rather than USB audio or PCM, to the built-in speaker.
Keep `g1_audio` registered because Speech validates its target in Atlas.

For frontend-only trials without USB input, set the greeting's
`voice_enabled: false` and restart it. In the same frontend session, send
“先去B，完成后继续去A” to test temporary work. `config/pilot_soul.md` keeps
ASR readiness from becoming a prerequisite for text navigation. Navigation
stopping, playback waiting, confirmed arm release and Stop remain required.

The top-level `env:` paths are explicit absolute paths: rbnx expands all env
values before setting sibling entries, so referencing `ROBONIX_DEPLOY_DIR`
there can give Pilot/nav2 different paths from the greeting skill. If moving
this deployment, update those paths together. After changing them, restart the
backend deployment so Pilot reloads its prompt and nav2's container is recreated
with the correct state-file mount; rebuilding is not required. Do not create
`/data/greeting_state.json` or clear greeting state to bypass this check.
Offline path checks (Python with PyYAML):
`python3 -B -m unittest discover -s tests -p test_deployment_config.py`.

## RViz local costmap

Use a Map display on `/local_costmap/costmap` with Reliable / Transient Local
QoS; keep `/local_costmap/costmap_updates` Reliable / Volatile. The G1 Nav2
parameters send full local grids continuously so RViz started after Nav2 can
initialize with a current timestamp. Otherwise the initial retained grid in
`odom` may predate RViz's `map -> odom` TF cache and be discarded before display,
even with the correct QoS. A temporary Fixed Frame of `odom` avoids that
transform when diagnosing this case. Parameter-file changes require restarting
Nav2 (or Robonix), without rebuilding the image.

The scan obstacle layers accept points at height zero: `/scanner/scan` is a
planar projection in `base_footprint`. Physical obstacle heights (0.20–1.50 m)
are filtered in `scan_projection` before projection; applying a positive
minimum height again to LaserScan discards those obstacle returns.

For obstacles that appear behind the G1 during rotation, collect floor tilt
and IMU measurements on a flat floor with the receive-only diagnostic script:

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=/data/fastdds_whitelist.xml
/usr/bin/python3 scripts/g1_floor_diagnostics.py --interactive
```

The terminal prompts for four headings. Release the remote-control stick and
settle for three seconds before each Enter; rotate about 90 degrees in the
same direction between samples. The script subscribes to `/scanner/cloud`,
`/livox/imu`, `/odom` and TF without sending commands. It saves transformed
clouds, floor heights, IMU gravity direction and odometry changes under a new
`data/nav-diagnostics/floor-TIMESTAMP/` directory. Use `--seconds 8 --label NAME`
for one heading or `--cloud-file PATH` to refit a saved cloud offline. Sampling
while moving invalidates comparison of floor and IMU gravity; check the saved
yaw span and angular velocity before using the result for calibration. Add
`--cloud-topic /scanner/cloud_aligned` to measure the cloud consumed by mapping
and navigation after gravity correction.

Both G1 profiles use Nav2's `obstacle_max_range` / `raytrace_max_range` keys:
5/8 m locally (clipped to the rolling grid) and 8/12 m globally. The old
`obstacle_range` / `raytrace_range` names left Nav2 at its 2.5/3 m defaults.
`observation_persistence: 0.0` keeps only the latest observation; it does not
expire obstacle cells. A later clearing ray must traverse a cell to clear it.

Clearing costmaps does not repair occupied cells in the RTAB-Map source map.
Range parameter corrections take effect after restarting navigation.

Before reproducing false obstacles, run the following with the same ROS
environment as the floor diagnostic above:

```bash
/usr/bin/python3 scripts/g1_record_navigation.py --seconds 120 --max-mib 2048
```

The receive-only recorder saves raw/aligned clouds, raw/filtered scans, IMU,
odometry, TF, `/map`, both costmaps, and available ground/obstacle clouds and
navigation status. It stops on duration, total size threshold, low disk space,
or Ctrl-C; final buffer flushing can slightly exceed the size threshold.
Results are in `data/nav-diagnostics/obstacles-TIMESTAMP/`. `status.json` lists
actual message counts and missing required topics; missing data causes a
nonzero exit. The copied config files are disk inputs, not proof of live
parameter values. Record before navigating and keep recording through the
false obstacle and manual turn so their timestamps can be compared.

Offline config checks: `python3 -B -m unittest discover -s tests -p test_deployment_config.py`.

## Environment Variables

| Variable | Default | Description |
|---|---|---|
| `G1_NETWORK_INTERFACE` | `lo` | Ethernet interface to the G1 robot |
| `G1_ALLOW_MOTION` | `false` | Enable motion commands (set `true` for supervised operation) |
| `G1_MAP_ID` | — | Map identifier for the current environment |
| `G1_MAP_MODE` | `localization` | `mapping` or `localization` |
| `SPEECH_BACKEND` | `local` | `local` (FunASR) or `tencent` (cloud ASR) |
| `G1_DASHBOARD_PORT` | `8092` | Dashboard web UI port |

The Speech config loads streaming FunASR weights directly from
`.cache/modelscope/models/iic/speech_paraformer-large_asr_nat-zh-cn-16k-common-vocab8404-online`
inside this deployment. Keep the complete model directory there before booting;
the client's Voice button uses this backend. Restarting `/data/robonix-client/bash.sh`
does not reload Speech; apply model configuration through its Driver or reboot Robonix.

## Camera over the wired G1 link

For direct Fast DDS reception, export both variables before `rbnx boot`:

```bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=/data/fastdds_whitelist.xml
```

The XML whitelist selects this machine's local interface address (currently
`192.168.123.220` on the laptop; `192.168.123.164` on G1). Scene, mapping and
Nav2 Docker launchers must mount the file and pass its environment variable;
host shell exports alone do not configure a container. The local launchers
now forward this file read-only. After restarting, verify each container with
`docker exec CONTAINER printenv FASTRTPS_DEFAULT_PROFILES_FILE`.

Measure one image topic at a time: each `ros2 topic hz` creates another
subscriber and can increase network load. If rates fall after boot, compare
UDP `RcvbufErrors` and IP `ReasmFails` in `/proc/net/snmp` over the same interval;
their cumulative totals alone cannot identify the current fault.

This deployment currently maps from LiDAR and IMU (`rtabmap_inputs`), so
camera reception alone does not establish that mapping works. Check
`/scanner/cloud`, `/livox/imu`, and `base_footprint -> livox_frame` TF.
Scene's spatial detections also require the camera TF to join the robot tree:
the external camera driver's `camera_link` must connect to the URDF's
`d435_link` using the verified mount convention. With `launch_realsense: false`,
the local primitive's `base_frame_id` setting does not reconfigure G1's driver.

Seeing a topic name does not prove that its image samples reach a subscriber.
The G1 SSH login prompt asking `ros:foxy(1) noetic(2)?` configures the host
shell, not the already running Humble camera container. Interrupting that menu
does not select Humble or diagnose DDS. Inspect the camera container directly:

```bash
docker inspect -f 'network={{.HostConfig.NetworkMode}} ports={{json .NetworkSettings.Ports}}' CONTAINER
docker exec CONTAINER bash -c 'printf "ROS_DISTRO=%s ROS_DOMAIN_ID=%s RMW_IMPLEMENTATION=%s ROS_LOCALHOST_ONLY=%s\n" "${ROS_DISTRO:-unset}" "${ROS_DOMAIN_ID:-unset}" "${RMW_IMPLEMENTATION:-unset}" "${ROS_LOCALHOST_ONLY:-unset}"'
```

Check the running camera process's environment too if the driver was launched
with `docker exec -e` or an entrypoint override: it may differ from a new
`docker exec` shell. Run ROS checks with `source /opt/ros/humble/setup.bash`
**inside the container**; do not source Foxy or Noetic on the G1 host for this
camera.

Before changing the transport, compare these commands **inside the camera
container** and on the laptop (with the same `ROS_DOMAIN_ID`):

```bash
ros2 topic info -v /camera/color/image_raw
ros2 topic echo --once --qos-reliability best_effort /camera/color/image_raw --field header
ros2 topic echo --once --qos-reliability best_effort /camera/aligned_depth_to_color/image_raw --field encoding
ros2 topic echo --once /camera/color/camera_info --field header
```

If the container itself has no frames, first check the RealSense process,
USB connection and camera logs: a network bridge cannot supply missing frames.
If only the laptop fails, check `ROS_LOCALHOST_ONLY`, the actual publisher QoS
reported by `ros2 topic info -v`, `ROS_DOMAIN_ID`, `RMW_IMPLEMENTATION`, Docker
network mode and the wired route/firewall. Image transport is much larger than
topic discovery, so discovery alone does not prove cross-host delivery. Stop
other camera publishers before testing, to avoid mistaking their graph entries
for the G1 driver.

The following HTTP path avoids cross-host DDS for camera frames. Do not use a
`web_video_server` MJPEG/PNG image as metric depth: it converts `16UC1` into a
display image. The HTTP path below preserves the source ROS image bytes and
timestamps for RGB and both depth topics.

On G1, check that `/camera/depth/image_rect_raw` and
`/camera/aligned_depth_to_color/image_raw` are both `16UC1`, and that
`/camera/color/camera_info` and `/camera/depth/camera_info` publish calibration.
Copy `scripts/g1_camera_http.py` from this deployment to the G1, then copy it
into the **existing** `d435i-ros2:humble` container and run it there alongside
the camera driver (replace `CONTAINER` with `docker ps`'s container ID):

```bash
docker cp /tmp/g1_camera_http.py CONTAINER:/tmp/g1_camera_http.py
docker exec -it CONTAINER bash -lc \
  'source /opt/ros/humble/setup.bash && python3 /tmp/g1_camera_http.py --host 0.0.0.0 --port 8081'
```

If you already have a shell **inside** the G1 camera container, you can copy
the script without SSH. First serve only the scripts directory on the laptop
over the trusted wired link, then fetch it from the G1 container and run it:

```bash
# Laptop, in a separate terminal; replace with an IP reachable from G1.
python3 -m http.server 8765 --bind LAPTOP_IP \
  --directory /data/robonix/examples/robot-unitree-g1/scripts

# G1 camera container (separate shell; web_video_server keeps running).
curl -fsS http://LAPTOP_IP:8765/g1_camera_http.py -o /tmp/g1_camera_http.py
python3 /tmp/g1_camera_http.py --host 0.0.0.0 --port 8081
```

Give the HTTP process the camera driver's actual `ROS_DOMAIN_ID`,
`RMW_IMPLEMENTATION` and `ROS_LOCALHOST_ONLY` settings if these were set only
for the driver process. Do not enable `ROS_LOCALHOST_ONLY=1` for the HTTP
process alone without verifying that it can receive the driver's images.

Stop the temporary laptop file server after the copy. Ensure the G1 HTTP
address is reachable before starting the laptop publisher; if the wired link
or route is down, neither the original RGB stream nor this new endpoint works.

The container must already use host networking or expose port 8081 to the G1
host; an existing Docker container cannot gain a published port via `docker
exec`. Verify from the laptop with `curl -i http://G1_IP:8081/color`; a 503
means there is no fresh local image, while connection refusal means the port is
unreachable. Restrict port 8081 to the trusted wired network: HTTP is not
authenticated. This read-only endpoint sends losslessly compressed native RGB,
both depth images and original color/depth CameraInfo without cross-host DDS.

On the laptop, stop any previous `all_test.py` process so it does not publish
duplicate camera topics, then start the tracked replacement:

```bash
G1_CAMERA_URL=http://192.168.123.164:8081 \
python3 scripts/http_to_ros_camera.py
```

Run the laptop command from this deployment directory with ROS 2 Humble and
`rclpy` available. Use the same laptop ROS environment for this publisher and
Robonix; isolate it from G1 DDS (for example, use `ROS_LOCALHOST_ONLY=1` for
both, provided all other local Robonix ROS nodes use this setting). Check:

```bash
ros2 topic echo --once --qos-reliability best_effort /camera/color/image_raw --field header
ros2 topic echo --once --qos-reliability best_effort /camera/depth/image_rect_raw --field encoding
ros2 topic echo --once --qos-reliability best_effort /camera/aligned_depth_to_color/image_raw --field encoding
ros2 topic echo --once /camera/color/camera_info --field header
```

Both depth encodings must remain `16UC1`; Scene uses the aligned depth and
color CameraInfo for object detection. The depth CameraInfo is also forwarded
for consumers of the unaligned depth topic. All three images retain G1's ROS
timestamps and optical frames; polling the latest RGB and depth independently
does not guarantee a matched pair. Consumers requiring synchronized pairs must
check header timestamps and reject overly separated frames.

The deployment sets the camera primitive's `launch_realsense: false`: run the
HTTP-to-ROS publisher before `rbnx boot`, or the primitive's RGB, aligned-depth
and color-CameraInfo startup checks will time out. It declares locally
republished RGB, aligned depth and color CameraInfo through Atlas. The HTTP
bridge does **not** carry
`/camera/extrinsics` or `/tf_static`; Scene needs a measured mount transform
(`extrinsics` in the camera primitive config) or a valid local TF tree.
Do not guess a camera mount transform. Avoid mixing the original remote DDS
publishers and the local bridge on the same topic: isolate the laptop ROS graph
from the G1 graph when using this bridge.

## Architecture

```
[ROS2 /cmd_vel] → g1_chassis_adapter → Unix socket → g1_loco_daemon → SDK2 LocoClient → G1
```

- **g1_chassis** — Guarded locomotion primitive with 300 ms watchdog, velocity clamping, and motion gating.
- **mid360_lidar / mid360_imu** — Head-mounted Livox Mid-360 3D LiDAR and IMU (UDP direct read).
- **realsense_camera** — Head-mounted Intel RealSense D435i depth camera for RGB-D mapping.
- **mapping** — RTAB-Map with Mid-360 LiDAR + Realsense depth fusion.
- **nav2** — Nav2 with bipedal-safe scan projection (tall torso clearance).
- **speech** — FunASR wake-word detection + Azure TTS.
- **g1_dashboard** — Operator web UI with camera stream, map, and navigation controls.

## Safety

- Motion is **disabled by default**. Requires `G1_ALLOW_MOTION=true`.
- 300 ms watchdog — missing cmd_vel triggers automatic StopMove.
- Velocity limits clamped in the daemon (vx ≤ 1.0 m/s, ω ≤ 2.0 rad/s).
- No grasp/IK pipeline — arms accept joint commands only.

## Configuration Files

| File | Purpose |
|---|---|
| `robonix_manifest.yaml` | Top-level deployment manifest |
| `soma.yaml` | Robot body description, component tree, capabilities |
| `config/nav2_params_g1.yaml` | Nav2 planner, controller, and costmap parameters |
| `config/rtabmap_params.yaml` | RTAB-Map mapping parameters |
| `config/navigate.xml` | Nav2 BT behavior tree for navigation goals |
| `config/navigate_through_poses.xml` | BT tree for multi-waypoint navigation |
| `config/g1_topics.yaml` | ROS2 topic registry (inputs, outputs, TF frames) |
| `config/robonix_client_settings.yaml` | Robonix Client workspace template |
| `config/semantic_landmarks.yaml` | Semantic landmark definitions for Chinese navigation |
