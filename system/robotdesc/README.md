# Robotdesc - raw body YAML and URDF service

Migration steps and the complete file inventory are in
`docs/soma-to-robotdesc-migration.md` and `docs/soma-to-robotdesc-files.txt`.
After building the Rust binaries and generating World stubs, run
`/usr/bin/python3 system/robotdesc/scripts/smoke_runtime.py` from the repository
root to verify isolated boot, primitive lifecycle, body RPCs and World geometry.

Robotdesc loads a single robot's `robotdesc.yaml` and referenced URDF, then exposes them
to other Robonix components over gRPC. `rbnx boot` reads the same
`robonix_manifest.yaml` to decide which primitive and skill packages to run;
Robotdesc spawns them through `rbnx start` in two stages (see
`system/robotdesc/src/launcher.rs`).

Robotdesc preserves the self-description as raw YAML and URDF for consumers. It
also interprets the existing `robot.components` tree when normalizing a
`robonix/primitive/health/stream` frame into `RobotdescHealthSnapshot`; this keeps
health component paths and kinds aligned with the robot description. A fresh
health-primitive frame takes precedence; if its TTL expires, Robotdesc continues
publishing the ROS runtime-state fallback.

## Config

Robotdesc v2 uses a flat, four-key schema. Every field is required.

```yaml
src_endpoint: 127.0.0.1:50051   # src gRPC endpoint
listen:         127.0.0.1:50091   # robotdesc's own gRPC listen address
provider_id:    robotdesc              # src provider_id
robot_yaml:     ./robotdesc.yaml       # path to the single robot's robotdesc.yaml
```

Relative paths in `robot_yaml` are resolved from the config file's directory.
Fields can also be supplied on the CLI (`--src`, `--listen`, `--provider-id`,
`--robot-yaml`); CLI flags win over config-file values. `rbnx boot` uses this
CLI mode — it does not write a robotdesc config file on disk.

The robot's `robotdesc.yaml` and any files it references (URDF, package
`robonix_manifest.yaml` for stage 1/2 bring-up) must live somewhere Robotdesc can
read at startup.

## Run

Direct invocation for local testing:

```bash
cargo run -p robonix-robotdesc -- --config ./robotdesc.local.yaml
```

Or with an inline config JSON blob (what `rbnx boot` uses):

```bash
robonix-robotdesc \
  --src 127.0.0.1:50051 \
  --listen 127.0.0.1:50091 \
  --provider-id robotdesc \
  --robot-yaml /path/to/robot/robotdesc.yaml
```

At startup Robotdesc parses the robot YAML, loads the referenced URDF, spawns
primitive packages via `rbnx start` (stage 1), registers itself in Src, and
serves gRPC on `listen`. Skill packages are held until `rbnx boot` writes
`stage2\n` into the pipe on `$ROBONIX_ROBOTDESC_STAGE_FD` (stage 2). Robotdesc stops
every package it launched on SIGINT/SIGTERM.

`--log` sets Robotdesc's log file level (`debug`, `info`, `warn`, or `error`);
package stdout/stderr is written through log under `$ROBONIX_LOG_DIR` or
`./logs`.

## gRPC API

`robonix/system/robotdesc/get_yaml` returns the loaded raw Robotdesc YAML:

```srv
string robot_id  # empty = default robot
---
string robot_id
string yaml_text
```

`robonix/system/robotdesc/get_urdf` returns the loaded raw URDF XML and, on request,
the files referenced by URDF-local relative mesh or texture paths:

```srv
string robot_id  # empty = default robot
bool include_assets
---
string robot_id
string urdf_xml
UrdfAsset[] assets
```

Absolute browser URLs and `package://` references are not attached. Relative
resources must resolve below the directory containing the URDF. Robotdesc indexes
these paths at startup and reads the files only for
`get_urdf(include_assets=true)`, so a missing mesh does not prevent non-rendering
deployments from starting.

`robonix/system/robotdesc/footprint` returns the active robot's 2D collision
polygon, base frame, inscribed radius, and circumscribed radius. Generic
services such as World and Navigation consume this contract instead of
carrying robot-model dimensions of their own.

`robonix/system/robotdesc/get_health` and `robonix/system/robotdesc/health` expose the
latest normalized hardware state. Health primitive reading names use stable
Robotdesc paths such as `body/base/left_wheel`; optional controls append
`/driver_temp`, `/enabled`, `/communication_ok`, `/online`, or `/error`.
Top-level `HealthState` power fields are attached to the component whose
`type` is `battery`. Hardware topology comes only from `robot.components`;
Robotdesc does not infer deployment-specific joints or devices.

An empty request `robot_id` selects `default_robot`. If no `default_robot` is
configured and exactly one robot is loaded, Robotdesc selects that only robot.

## Robotdesc YAML Spec

Example:

```yaml
urdf:
  path: ./rover_arm.urdf
  root_link: base_link
  model_name: rover_arm

robot:
  id: rover_arm_01
  display_name: "four-wheel rover with 6-DOF arm"
  family: mobile_manipulator
  root_part: base
  dimensions: { length_m: 0.84, width_m: 0.56, height_m: 1.20 }
  mass_kg: 38
  passable_door_width_m: 0.78
  exports:
    - provider_id: rover_nav
      capabilities:
        - { path: robonix/service/navigation/navigate, description: "Navigate to a 2D goal." }
    - provider_id: skill_explore_room
      capabilities:
        - { path: robonix/skill/explore/room, description: "Explore the current room." }
  components:
    - id: base
      type: mobile_base
      urdf_link: base_link
      exports:
        - provider_id: rover_chassis
          capabilities:
            - { path: robonix/primitive/chassis/move, description: "Command chassis motion." }
            - { path: robonix/primitive/chassis/odom, description: "Read chassis odometry." }
      components:
        - id: left_wheel
          type: wheel
          urdf_joint: wheel_left_joint
          exports: []
    - id: head_camera
      type: rgbd_camera
      urdf_link: camera_optical_frame
      exports:
        - provider_id: rover_camera
          capabilities:
            - { path: robonix/primitive/camera/snapshot, description: "Capture an RGB image." }

description:
  summary: "Rover with chassis, RGB-D camera, and exploration skill."
  can_do: ["drive", "navigate", "capture RGB-D images"]
  cannot_do: ["manipulate objects"]
  notes: ["Robotdesc serves this YAML and the referenced URDF as raw text."]
```

### URDF

| Field | Type | Required | Description |
|---|---|---|---|
| `path` | string | yes | URDF path, relative to the Robotdesc YAML file directory unless absolute. |
| `root_link` | string | yes | Root link of this composite URDF. |
| `model_name` | string | no | Human-readable or simulator-facing model name. |

Robotdesc preserves the original URDF XML. `get_urdf()` returns that raw XML text
and can include its URDF-local resources for clients that need to render the
visual geometry.

#### URDF resource path convention

Robot descriptions intended for Robotdesc and Monitor must use URDF-local relative
paths for Mesh and texture resources. A recommended deployment layout is:

```text
robot-description/
├── robotdesc.yaml
└── model/
    ├── robot.urdf
    ├── meshes/       # prepared locally; normally not tracked by Git
    │   ├── stl/
    │   └── dae/
    └── textures/     # prepared locally; normally not tracked by Git
```

The `filename` value is resolved from the directory containing the URDF, not
from the repository root, current working directory, or `robotdesc.yaml` directory:

```xml
<mesh filename="meshes/stl/base_link.stl"/>
<mesh filename="meshes/dae/arm/link_1.dae"/>
<texture filename="textures/body.png"/>
```

For a portable Robotdesc-to-Client rendering path, every resource reference must:

- use `/` as the path separator;
- be relative to the URDF directory and remain below that directory;
- preserve enough of the upstream directory structure to avoid filename
  collisions;
- match the local filename exactly, including case;
- point to a readable file before a client requests `include_assets=true`.

Parent traversal such as `../meshes/link.stl` and symlinks that resolve outside
the URDF directory are rejected. Absolute filesystem paths and `package://`,
`file://`, `http://`, `https://`, or `data:` references are not attached to the
`get_urdf` response and therefore must not be used when the Client is expected
to render only from Robotdesc-provided resources.

Large binary Mesh and texture files should be prepared locally instead of
being committed to the Robonix repository. Each robot example that relies on
external resources should keep these text files in Git:

- the URDF containing the stable relative references;
- a README with the official source repository, exact tag or commit, license,
  and upstream-to-local path mapping;
- a download or generation command;
- a manifest of expected resource paths and checksums when available;
- `.gitignore` entries covering the locally prepared binary directories.

For example, if an official checkout contains
`piper/meshes/dae/link1.dae`, an example may place it at
`model/meshes/dae/link1.dae`; a URDF stored at `model/robot.urdf` must then use
`filename="meshes/dae/link1.dae"`. Robotdesc validates the relative reference at
startup, reads the file when `get_urdf(include_assets=true)` is called, and the
Client serves it through its same-origin URDF resource endpoint. The Client
does not clone model repositories or repair incorrect URDF paths.

### Robot

| Field | Type | Required | Description |
|---|---|---|---|
| `id` | string | yes | Body id. One Robotdesc YAML file describes one body, and this is the robot's unique identifier. |
| `display_name` | string | yes | Name used in natural-language descriptions. |
| `family` | string | yes | Robot family, such as `mobile_robot`, `mobile_manipulator`, `fixed_dual_arm_desktop`, or `drone`. Custom values are allowed. |
| `root_part` | string | no | Component id that represents the root body part. |
| `dimensions` | object | yes | Overall dimensions, usually with `length_m`, `width_m`, and `height_m`. |
| `mass_kg` | float | yes | Overall mass. |
| `passable_door_width_m` | float | no | Conservative door-width threshold. |
| `exports` | array | yes | Provider-grouped capabilities available at robot scope. |
| `components` | array | yes | Hierarchical semantic body components. |

### Exports

`exports` is a list of provider groups. Each group names one provider and the
capabilities that provider contributes at the current scope.

```yaml
exports:
  - provider_id: tiago_webots_chassis
    capabilities:
      - { path: robonix/primitive/chassis/move, description: "Command chassis motion." }
      - { path: robonix/primitive/chassis/odom, description: "Read odometry." }
  - provider_id: skill_explore_room
    capabilities:
      - { path: robonix/skill/explore/room, description: "Explore the current room." }
```

Robot-level `exports` are for whole-body service and skill providers. Component
`exports` are for providers attached to that body part, such as a chassis,
camera, lidar, audio IO, or arm provider.

| Field | Type | Required | Description |
|---|---|---|---|
| `provider_id` | string | yes | Provider id that exposes the listed capabilities. |
| `capabilities` | array | yes | Capabilities exposed by that provider. |
| `capabilities[].path` | string | yes | Capability contract path, such as `robonix/primitive/chassis/move`. |
| `capabilities[].description` | string | no | Short natural-language capability description. |

### Components

`components` is the semantic body tree for the composite URDF. It does not need
to repeat every URDF joint one by one; actual transforms still come from the
URDF link / joint tree.

```yaml
components:
  - id: base
    type: mobile_base
    urdf_link: base_link
    exports:
      - provider_id: rover_chassis
        capabilities:
          - { path: robonix/primitive/chassis/move, description: "Command chassis motion." }
    components:
      - id: left_wheel
        type: wheel
        urdf_joint: wheel_left_joint
        exports: []
```

| Field | Type | Required | Description |
|---|---|---|---|
| `id` | string | yes | Component id, unique within this Robotdesc YAML file. |
| `type` | string | yes | Component type. Common values include `mobile_base`, `wheel`, `battery`, `body_part`, `lidar_2d`, `rgb_camera`, `rgbd_camera`, and `audio_io`; custom values are allowed. |
| `urdf_link` | string | no | URDF link represented by this component. |
| `urdf_joint` | string | no | URDF joint represented by this component. |
| `exports` | array | yes | Provider-grouped capabilities attached to this component. Use `[]` when none are attached. |
| `components` | array | no | Child components. |

### Description

`description` is a general natural-language description block.

```yaml
description:
  summary: "Short deployment summary."
  can_do: ["can do 1", "can do 2"]
  cannot_do: ["cannot do 1"]
  notes: ["note 1"]
```

| Field | Type | Required |
|---|---|---|
| `summary` | string | no |
| `can_do` | array | no |
| `cannot_do` | array | no |
| `notes` | array | no |
