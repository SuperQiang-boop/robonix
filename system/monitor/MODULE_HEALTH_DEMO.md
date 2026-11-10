# Monitor Module Health Demo

This document demonstrates the minimal system-module health loop in Monitor:

```text
executor / planner self-report ModuleHealthReport
        -> Monitor polls get_health periodically
        -> Monitor aggregates ModuleHealthSnapshot
        -> Monitor synthesizes STALE / ERROR after ttl_ms expires
        -> Monitor records ERROR -> OK after the module reports again
```

No real robot hardware is required. The hardware-health path uses mock Robotdesc,
while the module-health path uses the executor and planner `get_health` services.

## 1. Prepare

Enter the repository and build the required binaries:

```bash
cd /path/to/robonix

cargo build -p robonix-src
cargo build -p robonix-executor
cargo build -p robonix-planner
cargo build -p robonix-monitor
```

Install `grpcurl` and `jq` if you want to query the gRPC snapshot directly.
Without `grpcurl`, the same flow can still be verified from Monitor logs.

## 2. Start The Stack

Use five terminals.

### Terminal 1: Src

```bash
cd /path/to/robonix

cargo run -p robonix-src -- \
  --listen 127.0.0.1:50051 \
  --capabilities capabilities
```

Src is the service registry and capability discovery service.

### Terminal 2: Executor

```bash
cd /path/to/robonix

cargo run -p robonix-executor -- \
  --src 127.0.0.1:50051 \
  --listen 127.0.0.1:50061
```

Executor declares:

```text
robonix/system/executor/get_health
```

Monitor polls this service.

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

The VLM fields may use dummy values for this demo because no task is submitted
to Planner. Only `get_health` is exercised.

Planner declares:

```text
robonix/system/planner/get_health
```

### Terminal 4: Mock Robotdesc

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

Mock Robotdesc declares:

```text
robonix/system/robotdesc/health
robonix/system/robotdesc/get_health
```

The `--mock-robotdesc-interval-ms 10000` setting emits one hardware-health frame
every 10 seconds, which keeps screen recordings readable.

### Terminal 5: Monitor

```bash
cd /path/to/robonix

cargo run -p robonix-monitor -- \
  --src 127.0.0.1:50051 \
  --listen 127.0.0.1:50091 \
  --thresholds-path system/monitor/thresholds/robotdesc_mock.yaml \
  --log robonix_monitor=info
```

Monitor declares:

```text
robonix/system/monitor/get
robonix/system/monitor/stream
robonix/system/monitor/modules/get
```

## 3. Observe Normal Startup

Monitor should print lines like:

```text
declared GetMonitor at 127.0.0.1:50091
declared StreamMonitor at 127.0.0.1:50091
declared ModuleHealthSnapshot at 127.0.0.1:50091
connected to Robotdesc provider 'mock-robotdesc' through Src
connected to Robotdesc StreamHealth
module health poll connected: executor (robonix/system/executor/get_health)
module health poll connected: planner (robonix/system/planner/get_health)
```

This confirms that both paths are alive:

```text
mock Robotdesc -> Monitor                       hardware-health path
executor / planner -> Monitor modules/get   module-health path
```

After about 10 seconds, Monitor also prints the mock Robotdesc body baseline:

```text
[monitor] body: NORMAL (computer_jetson/jetson_agx_orin)
[monitor] body: NORMAL (arm/mock_arm)
[monitor] body: NORMAL (battery_main/mock_bms)
```

Those body lines are hardware-health data, not module-health data.

## 4. Query ModuleHealthSnapshot

If `grpcurl` is installed, query the Monitor aggregate module-health snapshot:

```bash
cd /path/to/robonix

PROTO_DIR=$(ls -td target/debug/build/robonix-monitor-*/out | head -n1)

grpcurl -plaintext \
  -import-path "$PROTO_DIR" \
  -proto robonix_contracts.proto \
  -d '{}' \
  127.0.0.1:50091 \
  robonix.contracts.RobonixSystemMonitorModulesGet/GetModuleHealthSnapshot | jq
```

The normal response should include executor and planner:

```json
{
  "snapshot": {
    "schemaVersion": 1,
    "modules": [
      {
        "moduleKey": "executor",
        "moduleId": "executor",
        "providerId": "executor",
        "health": 0,
        "state": "active",
        "reasonCode": "OK",
        "detail": "executor serving",
        "source": "SELF_REPORTED",
        "ttlMs": 5000
      },
      {
        "moduleKey": "planner",
        "moduleId": "planner",
        "providerId": "planner",
        "health": 0,
        "state": "active",
        "reasonCode": "OK",
        "detail": "planner serving",
        "source": "SELF_REPORTED",
        "ttlMs": 5000
      }
    ]
  }
}
```

Field summary:

| Field | Meaning |
|---|---|
| `health = 0` | OK |
| `health = 1` | WARN |
| `health = 2` | ERROR |
| `source = SELF_REPORTED` | Reported by the module's own `get_health` service |
| `ttlMs = 5000` | The report is valid for 5 seconds |

## 5. Test Executor Stale

Keep Src, mock Robotdesc, Monitor, and Planner running.

Stop executor in Terminal 2:

```bash
Ctrl+C
```

Monitor first notices that polling failed:

```text
[monitor] module health poll lost executor: ...
```

If executor had previously reported successfully, Monitor synthesizes stale after
`ttlMs = 5000` expires:

```text
[monitor] module executor health: OK -> ERROR (STALE)
[monitor] ALERT: module executor - no health report received within ttl
```

Query `modules/get` again. Executor should now look like:

```json
{
  "moduleKey": "executor",
  "moduleId": "executor",
  "providerId": "executor",
  "health": 2,
  "state": "stale",
  "reasonCode": "STALE",
  "detail": "no health report received within ttl",
  "source": "MONITOR_SYNTHESIZED_STALE",
  "ttlMs": 5000
}
```

Monitor only synthesizes stale for modules that have reported at least once.
A module that never appeared is not immediately marked as ERROR.

## 6. Test Executor Recovery

Restart Terminal 2:

```bash
cd /path/to/robonix

cargo run -p robonix-executor -- \
  --src 127.0.0.1:50051 \
  --listen 127.0.0.1:50061
```

Monitor should print:

```text
[monitor] module health poll connected: executor (robonix/system/executor/get_health)
[monitor] module executor health: ERROR -> OK (OK)
```

Query `modules/get` again. Executor should recover to:

```json
{
  "moduleKey": "executor",
  "health": 0,
  "state": "active",
  "reasonCode": "OK",
  "source": "SELF_REPORTED"
}
```

## 7. Test Planner Stale

Planner uses the same flow:

1. Stop planner in Terminal 3.
2. Wait about 5 seconds.
3. Monitor should print `planner OK -> ERROR (STALE)`.
4. Restart planner.
5. Monitor should print `planner ERROR -> OK (OK)`.

## 8. Troubleshooting

### Monitor Does Not Print Executor / Planner Connected

Check that executor and planner are running and using the same Src endpoint:

```text
--src 127.0.0.1:50051
```

### Planner Fails Because VLM Config Is Missing

Planner currently requires VLM fields at startup even when only `get_health` is
tested. Use the dummy values shown in this demo.

### Port Already In Use

This demo uses:

| Service | Port |
|---|---|
| Src | `50051` |
| Executor | `50061` |
| Planner | `50071` |
| Monitor | `50091` |
| Mock Robotdesc | `50092` |

Stop the old process or replace all affected ports consistently.

### grpcurl Query Fails

First confirm that Monitor has been built and that the generated proto exists:

```bash
PROTO_DIR=$(ls -td target/debug/build/robonix-monitor-*/out | head -n1)
ls "$PROTO_DIR/robonix_contracts.proto"
```

If `grpcurl` is unavailable, verify the flow from Monitor logs instead.

## 9. What This Demo Proves

After completing the steps above, the demo proves that:

1. executor and planner expose `ModuleHealthReport` through the V1 protocol.
2. Monitor discovers and polls their `get_health` services.
3. Monitor exposes the aggregate `ModuleHealthSnapshot` through `modules/get`.
4. Monitor synthesizes `STALE / ERROR` when a known module exceeds `ttl_ms`.
5. Monitor records and exposes recovery when the module returns to `OK`.
