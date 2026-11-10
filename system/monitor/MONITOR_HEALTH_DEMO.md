# Monitor Health Demo

This runbook shows how to demo Monitor collecting health from two paths:

```text
hardware health:
  Robotdesc health source -> RobotdescHealthSnapshot -> MonitorSnapshot

software health:
  monitor / executor / planner -> ModuleHealthReport -> ModuleHealthSnapshot
```

The demo does not depend on whether the Robotdesc source is mock or real. The
commands below use mock Robotdesc by default because it is stable for recording and
does not require hardware. If a real Robotdesc deployment is available, replace only
Terminal 4; the Monitor side stays the same.

## Ports

Use one Src endpoint for every process:

| Process | Address |
|---|---|
| Src | `127.0.0.1:50051` |
| Executor | `127.0.0.1:50061` |
| Planner | `127.0.0.1:50071` |
| Robotdesc health source | `127.0.0.1:50092` |
| Monitor | `127.0.0.1:50091` |

## Build

```bash
cd /path/to/robonix

cargo build -p robonix-src
cargo build -p robonix-executor
cargo build -p robonix-planner
cargo build -p robonix-monitor
```

If you plan to use real Robotdesc instead of mock Robotdesc:

```bash
cargo build -p robonix-robotdesc
```

## Start The Demo

Use five terminals.

### Terminal 1: Src

```bash
cd /path/to/robonix

cargo run -p robonix-src -- \
  --listen 127.0.0.1:50051 \
  --capabilities capabilities
```

### Terminal 2: Executor

```bash
cd /path/to/robonix

cargo run -p robonix-executor -- \
  --src 127.0.0.1:50051 \
  --listen 127.0.0.1:50061
```

### Terminal 3: Planner

```bash
cd /path/to/robonix

cargo run -p robonix-planner -- \
  --src 127.0.0.1:50051 \
  --listen 127.0.0.1:50071 \
  --vlm-upstream http://127.0.0.1:9/v1 \
  --vlm-api-key dummy \
  --vlm-model dummy
```

The dummy VLM settings are enough for this demo because no Planner task is
submitted. Only Planner's `get_health` endpoint is exercised.

### Terminal 4: Robotdesc Health Source

Default, hardware-free demo source:

```bash
cd /path/to/robonix

cargo run -p robonix-monitor -- \
  --src 127.0.0.1:50051 \
  --mock-robotdesc \
  --mock-robotdesc-listen 127.0.0.1:50092 \
  --mock-robotdesc-scenario normal \
  --mock-robotdesc-interval-ms 10000 \
  --log info
```

This process declares the same Robotdesc health contracts that Monitor consumes:

```text
robonix/system/robotdesc/get_health
robonix/system/robotdesc/health
```

If you want to use real Robotdesc instead, run this in Terminal 4:

```bash
cd /path/to/robonix

cargo run -p robonix-robotdesc -- \
  --src 127.0.0.1:50051 \
  --robot-yaml /path/to/deploy/robotdesc.yaml \
  --listen 127.0.0.1:50092
```

For real hardware data, the deployment directory must contain a valid
`robotdesc.yaml`, a matching `robonix_manifest.yaml`, and a health primitive that
exposes `robonix/primitive/health/stream`.

### Terminal 5: Monitor

```bash
cd /path/to/robonix

cargo run -p robonix-monitor -- \
  --src 127.0.0.1:50051 \
  --listen 127.0.0.1:50091 \
  --thresholds-path system/monitor/thresholds/example_thresholds.yaml \
  --log robonix_monitor=info
```

Monitor declares:

```text
robonix/system/monitor/get
robonix/system/monitor/stream
robonix/system/monitor/modules/get
```

## What To Show

In the Monitor terminal, show these lines:

```text
[monitor] connected to Robotdesc provider ...
[monitor] connected to Robotdesc StreamHealth
[monitor] module health poll connected: executor (robonix/system/executor/get_health)
[monitor] module health poll connected: planner (robonix/system/planner/get_health)
```

These lines prove that both collection paths are alive:

```text
Robotdesc health source -> Monitor hardware health
executor / planner -> Monitor software module health
```

## Query Results

Use a sixth terminal.

Prepare the generated protobuf import path:

```bash
cd /path/to/robonix

PROTO_DIR=$(ls -td target/debug/build/robonix-monitor-*/out | head -n1)
```

### 1. Query Robotdesc Source Health

```bash
grpcurl -plaintext \
  -import-path "$PROTO_DIR" \
  -proto robonix_contracts.proto \
  -d '{}' \
  127.0.0.1:50092 \
  robonix.contracts.RobonixSystemRobotdescGetHealth/GetHealth | jq
```

Expected evidence:

- `snapshot.schemaVersion` is `1`.
- `snapshot.components` contains the hardware component tree.
- Mock Robotdesc returns deterministic demo values.
- Real Robotdesc returns hardware data from the configured health primitive.

### 2. Query Monitor Hardware Snapshot

```bash
grpcurl -plaintext \
  -import-path "$PROTO_DIR" \
  -proto robonix_contracts.proto \
  -d '{}' \
  127.0.0.1:50091 \
  robonix.contracts.RobonixSystemMonitorGet/GetMonitor | jq
```

Expected evidence:

- `snapshot.bodies` contains normalized hardware health from Robotdesc.
- Joint entries include stable component ids such as `body/arm/joint_1`.
- Temperature, enable state, and error-code fields are present.

### 3. Query Monitor Software Module Health

```bash
grpcurl -plaintext \
  -import-path "$PROTO_DIR" \
  -proto robonix_contracts.proto \
  -d '{}' \
  127.0.0.1:50091 \
  robonix.contracts.RobonixSystemMonitorModulesGet/GetModuleHealthSnapshot | jq
```

Expected evidence:

```text
monitor   OK active SELF_REPORTED
executor OK active SELF_REPORTED
planner    OK active SELF_REPORTED
```

This proves that Monitor is collecting software module health in the same demo.

## Optional Fault Demo

Stop Executor in Terminal 2. After the configured TTL, Monitor should log:

```text
[monitor] module health poll lost executor: ...
[monitor] module executor health: OK -> ERROR (STALE)
[monitor] ALERT: module executor - no health report received within ttl
```

Query module health again:

```bash
grpcurl -plaintext \
  -import-path "$PROTO_DIR" \
  -proto robonix_contracts.proto \
  -d '{}' \
  127.0.0.1:50091 \
  robonix.contracts.RobonixSystemMonitorModulesGet/GetModuleHealthSnapshot | jq
```

Executor should now have:

```text
health: ERROR
state: stale
reason_code: STALE
source: MONITOR_SYNTHESIZED_STALE
```

Restart Executor, and Monitor should log:

```text
[monitor] module health poll connected: executor (robonix/system/executor/get_health)
[monitor] module executor health: ERROR -> OK (OK)
```

## Common Pitfalls

- All processes must use the same Src endpoint.
- Do not mix `50051` and `50251` unless every process is changed together.
- The Robotdesc source can be mock or real; Monitor consumes the same contract either
  way.
- Real hardware data requires a health primitive in the deployment manifest.
- Monitor can start before Robotdesc is available and retries the Robotdesc stream in the
  background.
