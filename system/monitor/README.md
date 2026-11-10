# robonix-monitor

Monitors robot onboard health (CPU/GPU/NVMe temperature, voltage) and body (joint motors), evaluates thresholds, and reports via gRPC.

Monitor consumes the unified health stream (`RobotdescHealthSnapshot`) from Robotdesc
(real or mock). If Robotdesc is not ready yet, Monitor keeps serving and retries the
stream connection in the background.

## Architecture

Two parallel data paths coexist:

| Path | Use case | Flow |
|---|---|---|
| **Primitive → ROBOTDESC** (production) | Real ROBOTDESC manages the deployment | `health_piper` primitive → ROBOTDESC (aggregates + streams `RobotdescHealthSnapshot`) → Monitor |
| **Mock ROBOTDESC** (development / testing) | No real ROBOTDESC needed | `--mock-robotdesc` in-process gRPC server → Monitor |

### Primitive → ROBOTDESC path (real hardware)

The health data pipeline when using real ROBOTDESC:

```
Piper CAN0 → health_piper (Python Primitive, robonix_api)
  → gRPC: robonix/primitive/health/stream (HealthState frames)
    → ROBOTDESC (discovers via Src, maps HealthState → RobotdescHealthSnapshot)
      → gRPC: robonix/system/robotdesc/health (RobotdescHealthSnapshot stream)
        → Monitor (robotdesc_ingest.rs — threshold evaluation + broadcasting)
```

The `health_piper` primitive implements `robonix/primitive/health/stream` and
wraps `piper_sdk`. It is **not** part of the Robonix source tree — create it in your
deployment's `primitives/` directory following the [vendor onboarding guide](https://book.robonix.ai/integration-guide/vendor-onboarding.html).

### Mock ROBOTDESC path (no ROBOTDESC needed)

The `--mock-robotdesc` flag starts an embedded gRPC server inside the Monitor process that
serves `robonix/system/robotdesc/health` and `robonix/system/robotdesc/get_health`. Monitor
consumes it through the same `robotdesc_ingest.rs` pipeline as production.

Optionally, `--mock-robotdesc-arm piper` spawns `scripts/piper_bridge.py` as a subprocess
(stdin/stdout JSON) to merge real joint data into synthetic snapshots.

## Running

For a screen-recording-friendly end-to-end health demo, see
[`MONITOR_HEALTH_DEMO.md`](MONITOR_HEALTH_DEMO.md).

### With `rbnx boot` (recommended)

Declare Monitor in the deployment manifest's existing `system:` block:

```yaml
system:
  monitor:
    listen: 127.0.0.1:50092
    log: info
```

`rbnx boot` starts `robonix-monitor`, supplies the Src endpoint from
`system.src.listen`, and passes the complete Monitor block as manifest JSON.
Fields without standalone CLI flags, including `expected_modules`, can be set
in this block. Explicit CLI or environment values take precedence over the
manifest, followed by an optional YAML config file and compiled defaults.

```bash
rbnx boot -f /path/to/deploy/robonix_manifest.yaml
```

The Webots example uses `127.0.0.1:50093` because port `50092` is reserved for
voiceprint in that deployment.

### With real ROBOTDESC (production)

```bash
# Terminal 1: Src
robonix-src --listen 127.0.0.1:50051 --capabilities capabilities

# Terminal 2: ROBOTDESC (spawns health_piper primitive, starts health collection)
robonix-robotdesc --src 127.0.0.1:50051 \
  --robot-yaml <deploy>/robotdesc.yaml \
  --listen 127.0.0.1:50091

# Terminal 3: Monitor (auto-discovers ROBOTDESC health stream via Src)
robonix-monitor --src 127.0.0.1:50051 \
  --listen 127.0.0.1:50092 \
  --thresholds-path system/monitor/thresholds/example_thresholds.yaml \
  --log info
```

Verify:

```bash
# Check registrations
rbnx caps -v    # Should show: robotdesc [ACTIVE], monitor [ACTIVE], health_piper [ACTIVE]

# ROBOTDESC health snapshot (includes Piper joint data)
grpcurl -plaintext -d '{}' 127.0.0.1:50091 \
  robonix.contracts.RobonixSystemRobotdescGetHealth/GetHealth

# Monitor normalized snapshot
grpcurl -plaintext -d '{}' 127.0.0.1:50092 \
  robonix.contracts.RobonixSystemMonitorGet/GetMonitor
```

### With mock ROBOTDESC (development, no ROBOTDESC binary needed)

```bash
# Terminal 1: Src
cargo run --release -p robonix-src -- --log info

# Terminal 2: mock Robotdesc (scenarios: normal / ramp / fault / toggle / mixed)
cargo run --release -p robonix-monitor -- --log info \
  --mock-robotdesc \
  --mock-robotdesc-scenario ramp \
  --mock-robotdesc-interval-ms 1000

# Terminal 3: Monitor consuming the Robotdesc health stream
cargo run --release -p robonix-monitor -- --log info
```

### Mock Robotdesc with real hardware (bridge subprocess)

```bash
# Piper arm via CAN bus
robonix-monitor --log info \
  --mock-robotdesc \
  --mock-robotdesc-arm piper \
  --mock-robotdesc-piper-can can0 \
  --mock-robotdesc-bridge-python ~/roboarm/.venv/bin/python3 \
  --mock-robotdesc-interval-ms 1000

# Koch arm via Dynamixel serial
robonix-monitor --log info \
  --mock-robotdesc \
  --mock-robotdesc-arm koch \
  --mock-robotdesc-koch-port /dev/ttyUSB0 \
  --mock-robotdesc-bridge-python ~/roboarm/.venv/bin/python3 \
  --mock-robotdesc-interval-ms 1000
```

## CLI flags

| Flag | Env var | Default |
|------|---------|---------|
| `--src` | `ROBONIX_SRC_ENDPOINT` | `127.0.0.1:50051` |
| `--listen` | `ROBONIX_MONITOR_LISTEN` | `127.0.0.1:50091` |
| `--id` | `ROBONIX_MONITOR_PROVIDER_ID` | `monitor` |
| `--thresholds-path` | `ROBONIX_MONITOR_THRESHOLDS_PATH` | `thresholds/example_thresholds.yaml` |
| `--robotdesc-endpoint` | `ROBONIX_ROBOTDESC_ENDPOINT` | — |
| `--mock-robotdesc` | `ROBONIX_MONITOR_MOCK_ROBOTDESC` | `false` |
| `--mock-robotdesc-listen` | `ROBONIX_MONITOR_MOCK_ROBOTDESC_LISTEN` | `127.0.0.1:50092` |
| `--mock-robotdesc-scenario` | `ROBONIX_MONITOR_MOCK_ROBOTDESC_SCENARIO` | `normal` |
| `--mock-robotdesc-interval-ms` | `ROBONIX_MONITOR_MOCK_ROBOTDESC_INTERVAL_MS` | `10000` |
| `--mock-robotdesc-arm` | `ROBONIX_MONITOR_MOCK_ROBOTDESC_ARM` | `synthetic` |
| `--mock-robotdesc-piper-can` | `ROBONIX_MONITOR_MOCK_ROBOTDESC_PIPER_CAN` | `can0` (when arm=piper) |
| `--mock-robotdesc-koch-port` | `ROBONIX_MONITOR_MOCK_ROBOTDESC_KOCH_PORT` | `/dev/ttyUSB0` (when arm=koch) |
| `--mock-robotdesc-bridge-python` | `ROBONIX_MONITOR_MOCK_ROBOTDESC_BRIDGE_PYTHON` | `python3` |
| `--mock-robotdesc-piper-script` | `ROBONIX_MONITOR_MOCK_ROBOTDESC_PIPER_SCRIPT` | `<crate>/scripts/piper_bridge.py` |
| `--mock-robotdesc-koch-script` | `ROBONIX_MONITOR_MOCK_ROBOTDESC_KOCH_SCRIPT` | `<crate>/scripts/koch_bridge.py` |
| `--config` | `ROBONIX_CONFIG_PATH` | — |
| `--config-json` | - | `system.monitor` block supplied by `rbnx boot` |
| `--log` | `RUST_LOG` | `robonix_monitor=info` |

## Threshold format

Robotdesc selector format:

```yaml
rules:
  - id: "joint_motor_temp"
    selector:
      kind: "JOINT"
      signal: "motor_temp"
    warn_above: 60.0
    error_above: 75.0
    unit: "degC"
```

## Testing

### Unit tests

```bash
# Monitor crate tests (mock scenarios, threshold evaluation, etc.)
cargo test -p robonix-monitor

# Full workspace tests
cargo test --workspace --all-targets
```

### End-to-end test with real Piper arm (CAN0)

Prerequisites: Piper arm connected via CAN0, `~/roboarm/.venv` with `piper-sdk>=0.6.1`,
and a `health_piper` primitive package created in your deployment per the
[vendor onboarding guide](https://book.robonix.ai/integration-guide/vendor-onboarding.html).

**Step 1: Create the primitive** in your deployment directory (e.g. `~/roboarm/robonix/primitives/health_piper/`), then run `rbnx codegen -p .` inside it.

**Step 2: Edit `system/monitor/tests/fixtures/robonix_manifest.yaml`** — add your primitive to the `primitive:` list:

```yaml
primitive:
  - name: health_piper
    path: /home/xjy/roboarm/robonix/primitives/health_piper   # absolute path (no ~)
    config:
      can_port: can0
```

**Step 3: Start the stack** (three terminals, order doesn't matter — Monitor waits for ROBOTDESC)

```bash
# Terminal 1: Src
cargo run --release -p robonix-src -- \
  --listen 127.0.0.1:50051 --capabilities capabilities

# Terminal 2: Monitor (starts gRPC server immediately, retries ROBOTDESC in background)
cargo run --release -p robonix-monitor -- \
  --src 127.0.0.1:50051 \
  --listen 127.0.0.1:50092 \
  --thresholds-path system/monitor/thresholds/example_thresholds.yaml \
  --log info

# Terminal 3: ROBOTDESC (spawns health_piper, starts health streaming)
cargo run --release -p robonix-robotdesc -- \
  --src 127.0.0.1:50051 \
  --robot-yaml system/monitor/tests/fixtures/robotdesc.yaml \
  --listen 127.0.0.1:50091
```

**Step 4: Verify**

```bash
# ROBOTDESC health snapshot — should contain Piper joint actuator data
grpcurl -plaintext -d '{}' 127.0.0.1:50091 \
  robonix.contracts.RobonixSystemRobotdescGetHealth/GetHealth

# Monitor normalized snapshot — should contain Piper arm body health
grpcurl -plaintext -d '{}' 127.0.0.1:50092 \
  robonix.contracts.RobonixSystemMonitorGet/GetMonitor
```

Expected: `GetMonitor` returns a `MonitorSnapshot` with `bodies[0]` containing
6 Piper arm joints with real motor temperatures read from CAN0.

### gRPC verification (mock ROBOTDESC)

```bash
# Monitor side
grpcurl -plaintext -d '{}' 127.0.0.1:50091 \
  robonix.contracts.RobonixSystemMonitorGet/GetMonitor

# Mock Robotdesc side
grpcurl -plaintext -d '{}' 127.0.0.1:50092 \
  robonix.contracts.RobonixSystemRobotdescGetHealth/GetHealth
```
