# Vitals → Monitor 命名迁移记录

## 目标与边界

记录日期：2026-11-09。本次把系统组件 `Vitals` 的运行时身份整体迁移为 `Monitor`：目录、Rust crate/binary、Src 注册身份、capability namespace、contract id、IDL 目录与消息名、gRPC 方法名、环境变量、manifest 键、CLI 参数翻译、示例、CI 与文档同步改名；不保留旧 crate、旧 contract id、旧生成名或兼容 alias。

迁移不改变健康数据的业务语义：`PowerState`、`ComponentHealth`、`BodyHealth`、`BodyComponent`、`ModuleHealth*` 及 Robotdesc 健康流仍按原字段工作，也不改变 `--mock-robotdesc*` 参数（它们描述 Robotdesc，而非本组件）。

工作树此时已包含 Atlas→Src、Pilot→Planner、Liaison→Gateway、Scene→World、Soma→Robotdesc 的未提交改动（基线 HEAD `f6850dcd`，分支 `rename`）。本轮只在这些改动之上改 Vitals 身份，未执行 `git clean`、未回滚、未提交、未创建分支。

## 名称映射

| 范围 | 旧名 | 新名 |
|---|---|---|
| 系统目录 | `system/vitals` | `system/monitor` |
| Cargo package / binary | `robonix-vitals` | `robonix-monitor` |
| Rust 日志 target / 前缀 | `robonix_vitals` | `robonix_monitor` |
| 系统 provider 默认 id | `vitals` | `monitor` |
| capability namespace | `robonix/system/vitals` | `robonix/system/monitor` |
| contract id | `robonix/system/vitals/{get,stream,modules/get}` | `robonix/system/monitor/{get,stream,modules/get}` |
| package name | `com.robonix.system.vitals` | `com.robonix.system.monitor` |
| IDL 根目录 | `capabilities/lib/vitals` | `capabilities/lib/monitor` |
| 系统 contract 目录 | `capabilities/system/vitals` | `capabilities/system/monitor` |
| RPC / 消息 | `GetVitals` / `StreamVitals` / `VitalsSnapshot` | `GetMonitor` / `StreamMonitor` / `MonitorSnapshot` |
| 生成类型 | `RobonixSystemVitalsGet` / `robonix_system_vitals_*` | `RobonixSystemMonitorGet` / `robonix_system_monitor_*` |
| Rust 类型 | `VitalsConfig` / `VitalsServiceImpl` / `VitalsState` | `MonitorConfig` / `MonitorServiceImpl` / `MonitorState` |
| 常量 | `DEFAULT_VITALS_PROVIDER_ID` / `VITALS_NAMESPACE` / `SOURCE_VITALS_SYNTHESIZED_STALE` | `DEFAULT_MONITOR_PROVIDER_ID` / `MONITOR_NAMESPACE` / `SOURCE_MONITOR_SYNTHESIZED_STALE` |
| 环境变量 | `ROBONIX_VITALS_*` | `ROBONIX_MONITOR_*` |
| manifest 键 | `system.vitals` | `system.monitor` |
| CLI 内置映射 | `("vitals", "robonix-vitals")` | `("monitor", "robonix-monitor")` |
| Makefile 目标 | `build-vitals` | `build-monitor` |
| 示例目录 | `examples/piper_vitals` | `examples/piper_monitor` |
| CI 标签 | `comp:vitals` | `comp:monitor` |
| 演示文档 | `VITALS_HEALTH_DEMO.md` | `MONITOR_HEALTH_DEMO.md` |

## 修改范围

- **IDL 与 contracts**：`capabilities/lib/vitals/` → `capabilities/lib/monitor/`（`srv/GetVitals.srv`→`srv/GetMonitor.srv`、`srv/StreamVitals.srv`→`srv/StreamMonitor.srv`、`msg/VitalsSnapshot.msg`→`msg/MonitorSnapshot.msg`，其余消息文件随目录搬迁）；`capabilities/system/vitals/` → `capabilities/system/monitor/`，三个 contract 的 `id` 与 `idl` 路径改为 monitor。
- **Rust 组件**：`system/vitals/` → `system/monitor/`，`Cargo.toml`、`build.rs`、`src/*.rs`、`tests/fixtures/*`、`scripts/*.py`、README 与三份演示文档全部使用新身份；`Cargo.toml` workspace member 与 `Cargo.lock` 同步。
- **工具链**：`tools/rbnx` 的 `BUILTIN_SYSTEM_BINARIES`、`system_cli_args` 的 `"monitor"` 分支、boot 帮助文案、注释与相关测试；根 `Makefile` 的目标/帮助；`tools/rbnx/README.md`。
- **调用方与文档**：`system/src`、`system/chronos`、`system/sentinel`、`system/robotdesc`（README 与 `src/health.rs` 注释）、`system/executor/src/main.rs` 注释、根 `README.md`、`MAINTAINERS.md`。
- **CI/治理**：`.github/CODEOWNERS`、`labeler.yml`、`labels.yml`、`issue-triage.yml`、issue 模板。
- **示例**：`examples/piper_vitals` → `examples/piper_monitor`（manifest 名 `robonix-piper-monitor-demo`），以及 Webots、Scout2、Agilex Ranger 的 manifest/README/stop 脚本。
- 逐文件旧→新清单见 `system/monitor/docs/vitals-to-monitor-files.txt`。

## 执行过程

1. 保存基线：分支、HEAD、`git status --short`（719 条，含此前迁移改动），仅记录不清理。
2. 全仓盘点 `vitals`（大小写不敏感）共 84 个文件，按运行时源码、IDL/contract、配置/CLI、生成物、备份、历史文档分类；`rbnx-build-bak-0814`、`.cache`、`CHANGELOG.md`、`docs/*-migration.md`、`system/robotdesc/docs/*` 明确排除。
3. 先迁目录与 IDL 文件，再改 IDL 内容与 contract `id`/`idl`；确认 proto package 变为 `robonix.monitor`。
4. 改 crate 元数据、workspace、`Cargo.lock`、`build.rs` 与全部 Rust 标识符（含生成 service/trait 名与 `--config-json` 解析）。
5. 改 `rbnx` 内置映射与参数翻译、Makefile、CI、维护者表、示例 manifest 和启动脚本；`--mock-robotdesc*` 参数保持原名。
6. 由源 IDL/contract 重新生成：`rbnx codegen -p system/world --mcp --clean --python /usr/bin/python3`（72 个 Python 模块，生成 `monitor_pb2*`，`proto_gen` 中已无 `vitals_pb2*`）。
7. 活跃源码旧名审计，剩余命中仅为历史文档、备份树、第三方缓存与上一轮迁移留下的生成目录。
8. 运行 fmt、workspace check/test、World pytest、真实进程注册与 gRPC 调用验收。

## 关键运行链路

- `rbnx boot` 解析到 `system.monitor` 后启动 `robonix-monitor`，把 manifest 块翻译为 `--listen/--src/--id/--thresholds-path/--robotdesc-endpoint/--log` 并附带 `--config-json`。
- Monitor 以 `provider_id=monitor` 注册 `robonix/system/monitor`，声明 `get`、`stream`、`modules/get` 三个 gRPC capability，并进入 ACTIVE。
- 健康数据仍来自 Robotdesc `StreamHealth`（真实或 `--mock-robotdesc` 内嵌 mock），经阈值规则转换为 `MonitorSnapshot`；自身模块健康以 `module_id=monitor` 进入聚合视图，超时来源标记为 `MONITOR_SYNTHESIZED_STALE`。
- 配置优先级不变：编译默认值 < YAML < manifest JSON < CLI/env；只把 `ROBONIX_VITALS_*` 换成 `ROBONIX_MONITOR_*`。

## 验证结果

已通过：

```text
cargo fmt --all -- --check
cargo check --workspace --all-targets
cargo test -p robonix-monitor            48 passed
cargo test -p robonix-cli                21 passed
cargo test --workspace --all-targets     272 passed, 0 failed
rbnx codegen -p system/world --mcp --clean --python /usr/bin/python3
                                         72 modules; monitor_pb2* present, vitals_pb2* absent in proto_gen
World pytest                             244 passed
```

真实进程验收（隔离端口 `15251`/`15291`、mock Robotdesc 监听 `15292`，`/usr/bin/python3` + grpc_tools 生成的新 proto stub）：

```text
rbnx caps -v:
  monitor [ACTIVE] robonix/system/monitor
    └─ robonix/system/monitor/get
    └─ robonix/system/monitor/stream
    └─ robonix/system/monitor/modules/get
rbnx contracts -p robonix/system/vitals → no contracts loaded
GetMonitor → bodies=3 components=16 voltage=24.20
GetModuleHealthSnapshot → monitor=SELF_REPORTED, executor/planner=MONITOR_SYNTHESIZED_STALE（未运行，预期）
```

生成 proto 中 `RobonixSystemMonitorGet.GetMonitor`、`RobonixSystemMonitorStream.StreamMonitor` 与 `robonix.monitor.MonitorSnapshot` 均正确，且已无 `vitals` 包。

## 旧名审计与保留项

- 活跃源码、manifest、脚本、CI、contract、生成输入中已无 `vitals` 身份；`rbnx contracts -p robonix/system/vitals` 返回空。
- 保留历史：`CHANGELOG.md` 的 Vitals 条目、`docs/*-migration.md` 与 `system/{world,robotdesc}/docs/*` 的历史快照、`rbnx-build-bak-0814` 备份生成物、`examples/robot-unitree-g1/.cache` 中第三方 `torch/ATen/core/Vitals.h`。
- `system/world/rbnx-build/codegen/scene_proto_gen/` 是 Scene→World 迁移遗留的旧生成目录（含 `vitals_pb2`），`rbnx codegen --clean` 不覆盖它；属构建产物，未手改，也未计入本轮源码。
- 语义名称保留：`health`、`module_health`、`RobotdescHealthSnapshot`、`PowerState`、`BodyHealth` 等描述数据/来源，不随组件改名；`--mock-robotdesc*` 与 `ROBONIX_ROBOTDESC_*` 同理。

## 未完成或受环境阻塞项

- `cargo clippy --workspace --all-targets -- -D warnings` 仍被工作树既有 `system/executor/src/service.rs:220/242/252/253` 的 4 个告警阻断（collapsible_if、`or_insert_with(String::new)`），非本轮迁移引入，未顺手修改无关控制逻辑。
- 未执行真实硬件（Piper/Koch 桥、G1、Scout2、Webots 整机 `rbnx boot`）、Docker 镜像构建、客户端 GUI 的 Vitals 页面联调；这些在无硬件/网络时不可验收。
- `examples/robot-unitree-g1` 的 `rbnx-boot/cache` 等 upstream 副本未改名；如上游包引用旧 contract id，需在各自仓库先改并发布，再更新部署 revision，不能只改 cache。
- 全部改动未提交、未推送；`docs/` 为未初始化子模块，根目录记录副本不被主仓库跟踪，主记录保存在 `system/monitor/docs/`。

## 可重复验收命令

```bash
cd /data/robonix
cargo fmt --all -- --check
cargo check --workspace --all-targets
cargo test -p robonix-monitor
cargo test --workspace --all-targets
cargo build -p robonix-src -p robonix-monitor -p robonix-cli

# 隔离进程验收（mock Robotdesc）
./target/debug/robonix-src --listen 127.0.0.1:15251 --capabilities "$PWD/capabilities" --log info &
./target/debug/robonix-monitor --mock-robotdesc --mock-robotdesc-listen 127.0.0.1:15292 &
./target/debug/robonix-monitor --src 127.0.0.1:15251 --listen 127.0.0.1:15291 \
  --robotdesc-endpoint 127.0.0.1:15292 --log info &
./target/debug/rbnx caps --server 127.0.0.1:15251 -v
./target/debug/rbnx contracts --server 127.0.0.1:15251 -p robonix/system/vitals   # 应为空
```

## 文档与提交状态

- 本记录：`system/monitor/docs/vitals-to-monitor-migration.md`，根 `docs/` 下为同步副本。
- 逐文件映射：`system/monitor/docs/vitals-to-monitor-files.txt`（47 条 move、31 条 update）。
- 本轮没有 `git commit`、`git push` 或创建分支。
