# Monitor Robotdesc Mock Demo

Monitor' `--mock-robotdesc` starts a full mock Robotdesc gRPC server in-process, streaming
`RobotdescHealthSnapshot`. Monitor consumes its StreamHealth via loopback gRPC —
**the processing pipeline is identical to production mode**. No real hardware
or real Robotdesc needed for development and debugging.

Optionally, `--mock-robotdesc-arm` selects a hardware backend: `synthetic` (default),
`piper` (CAN bus via piper_sdk), or `koch` (Dynamixel serial via dynamixel_sdk).
When a real backend is chosen, mock Robotdesc spawns the corresponding bridge
subprocess to read real joint data and merge it into the synthetic snapshot.

## Deployment topology

```
┌─────────────────────────────────────────────┐
│ mock_robotdesc.rs (in Monitor process)             │
│   5 scenarios producing deterministic mock   │
│   data                                       │
│                                              │
│   Optional: hardware bridge subprocess       │
│   → real arm data merged into snapshot       │
│   gRPC server on :50092                      │
└──────────────┬──────────────────────────────┘
               │ StreamHealth (gRPC, loopback)
               v
┌─────────────────────────────────────────────┐
│ Monitor (same process)                       │
│   robotdesc_ingest.rs: normalize + threshold eval │
│   service.rs: GetMonitor + StreamMonitor       │
│   gRPC server on :50091                      │
└──────────────────────────────────────────────┘
```

## Quick start

Three terminals to run the full health pipeline:

```bash
# Terminal 1: Src
target/debug/robonix-src \
  --listen 127.0.0.1:50251 \
  --capabilities /path/to/robonix/capabilities

# Terminal 2: mock Robotdesc (scenarios: normal / ramp / fault / toggle / mixed)
target/debug/robonix-monitor \
  --mock-robotdesc \
  --src 127.0.0.1:50251 \
  --mock-robotdesc-listen 127.0.0.1:50292 \
  --mock-robotdesc-scenario mixed \
  --mock-robotdesc-interval-ms 10000

# Terminal 3: Monitor consuming the Robotdesc health stream
target/debug/robonix-monitor \
  --src 127.0.0.1:50251 \
  --listen 127.0.0.1:50291 \
  --thresholds-path system/monitor/thresholds/example_thresholds.yaml \
  --log info
```

## Optional: real hardware bridges

### Piper arm (CAN bus)

```bash
target/debug/robonix-monitor \
  --mock-robotdesc \
  --src 127.0.0.1:50251 \
  --mock-robotdesc-arm piper \
  --mock-robotdesc-piper-can can0 \
  --mock-robotdesc-bridge-python /path/to/roboarm/.venv/bin/python3 \
  --mock-robotdesc-interval-ms 500
```

When `--mock-robotdesc-arm=piper`:
- Spawns `piper_bridge.py` subprocess, reads real joint data via piper_sdk
- Replaces matching ActuatorState by `joint_1..6` name on `body/arm`
- `PiperData.state` → SafetyState.aggregate_state
- Non-zero error_code → appended FaultState

### Koch arm (Dynamixel serial)

```bash
target/debug/robonix-monitor \
  --mock-robotdesc \
  --src 127.0.0.1:50251 \
  --mock-robotdesc-arm koch \
  --mock-robotdesc-koch-port /dev/ttyUSB0 \
  --mock-robotdesc-bridge-python /path/to/roboarm/.venv/bin/python3 \
  --mock-robotdesc-interval-ms 500
```

When `--mock-robotdesc-arm=koch`:
- Spawns `koch_bridge.py` subprocess, reads real joint data via dynamixel_sdk
- Replaces matching ActuatorState by `joint_1..6` name on `body/arm`
- `KochData.state` → SafetyState.aggregate_state (NORMAL=0, FAULT=1)
- Non-zero Hardware_Error_Status → appended FaultState

## Mock scenarios

| Scenario | Behavior |
|----------|----------|
| `normal` | All joint temps 36-41°C, communication OK, no faults |
| `ramp` | joint_1 temp ramps linearly from 40°C to 88°C every 30s, crossing WARN(60°C)→ERROR(75°C) |
| `fault` | joint_3 injected with overcurrent fault on 4 of every 8 cycles |
| `toggle` | joint_6 torque_enabled toggles on 4 of every 8 cycles |
| `mixed` | ramp + fault + toggle simultaneously |

## Simulated robot

The mock snapshot represents a small robot with a Jetson compute platform, one arm, and battery:

```
body
  computer_jetson
    cpu
    gpu
  arm (model varies by --mock-robotdesc-arm)
    joint_1 .. joint_6
  battery_main
```

Each joint publishes actuator data (position/velocity/effort/motor_temp/driver_temp etc.),
the battery publishes soc/voltage/current/temperature, and Jetson publishes cpu/gpu temps and fan_rpm.

The arm component model depends on the selected backend:
- `synthetic` → `mock_arm` / `mock_motor`
- `piper` → `piper` / `piper_motor`
- `koch` → `koch` / `dynamixel_motor`

## Expected log output

Using the `mixed` scenario:

```
# Temperature threshold changes
body/arm/joint_1/motor_temp health: OK -> WARN
body/arm/joint_1/motor_temp health: WARN -> ERROR

# Explicit faults
ALERT: body/arm/joint_3/fault/overcurrent - mock joint_3 overcurrent

# Enable state changes
body/arm/joint_6 enabled: true -> false

# Body state changes
arm/mock_arm body state: NORMAL -> FAULT

# Snapshot summary
24.2V | body/arm/joint_1/motor_temp:OK(40) ...
```

## Check registrations

```bash
ROBONIX_SRC=127.0.0.1:50251 target/debug/rbnx caps -v
```

Expected output:

```
mock-robotdesc [ACTIVE] robonix/system/robotdesc
monitor [ACTIVE] robonix/system/monitor
```

## gRPC verification

```bash
# Monitor side
grpcurl -plaintext -d '{}' 127.0.0.1:50091 \
  robonix.contracts.RobonixSystemMonitorGet/GetMonitor

# Mock Robotdesc side
grpcurl -plaintext -d '{}' 127.0.0.1:50092 \
  robonix.contracts.RobonixSystemRobotdescGetHealth/GetHealth
```

## Threshold file

Uses `thresholds/example_thresholds.yaml` (Robotdesc selector format):

```yaml
rules:
  - id: "joint_motor_temp"
    selector: { kind: "JOINT", signal: "motor_temp" }
    warn_above: 60.0
    error_above: 75.0
    unit: "degC"
  - id: "battery_soc"
    selector: { kind: "BATTERY", signal: "soc_percent" }
    warn_below: 20.0
    error_below: 8.0
    unit: "percent"
  # ...
```

`communication_ok = false`, `vendor_error_code != 0`, and active `fault_id` are
not evaluated through YAML thresholds — they are converted to WARN/ERROR directly
in `robotdesc_ingest.rs`.

## CLI flags

| Flag | Env var | Default |
|------|---------|---------|
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

## Implementation files

```
system/monitor/src/mock_robotdesc.rs     # MockRobotdescService + ArmBridge + generate_snapshot
system/monitor/src/subprocess.rs    # SubprocessHandle (bridge subprocess management)
system/monitor/scripts/piper_bridge.py    # PiperCollector + stdin/stdout JSON wrapper
system/monitor/scripts/koch_bridge.py     # KochCollector + stdin/stdout JSON wrapper
```
