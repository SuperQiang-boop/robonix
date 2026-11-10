# Soma → Robotdesc 命名迁移记录

## 迁移目标

本轮把系统组件 `Soma` 的运行时身份完整迁移为 `Robotdesc`，用于统一机器人本体描述、URDF、footprint 和健康聚合服务的目录、二进制、provider、contract、配置和调用方命名。迁移不改变 YAML/URDF 的业务字段、ROS 数据语义、Driver 生命周期顺序、Src 通信模型或 Robotdesc 对 primitive/skill 的两阶段启动行为。

记录日期：2026-11-07。健康消息中的组件时间戳字段随身份改为 `robotdesc_ts_ns`，因此旧生成代码和新服务不能混用；本轮不保留旧接口 alias。

工作树同时包含先前 Atlas→Src、Pilot→Planner、Liaison→Gateway、Scene→World 的未提交修改。本轮只在这些现有基础上修改 Soma 身份，不执行 `git clean`、不回滚、不提交、不创建分支。

## 名称映射

| 范围 | 旧名 | 新名 |
|---|---|---|
| 系统目录 | `system/soma` | `system/robotdesc` |
| Cargo package | `robonix-soma` | `robonix-robotdesc` |
| Rust library | `robonix_soma` | `robonix_robotdesc` |
| binary | `robonix-soma` | `robonix-robotdesc` |
| 系统 provider 默认 id | `soma` | `robotdesc` |
| capability namespace | `robonix/system/soma` | `robonix/system/robotdesc` |
| contract id 前缀 | `robonix/system/soma/*` | `robonix/system/robotdesc/*` |
| IDL 根目录 | `capabilities/lib/soma` | `capabilities/lib/robotdesc` |
| 系统 contract 目录 | `capabilities/system/soma` | `capabilities/system/robotdesc` |
| YAML 文件 | `soma.yaml` | `robotdesc.yaml` |
| 健康消息 | `SomaHealthSnapshot` | `RobotdescHealthSnapshot` |
| 时间戳字段 | `soma_ts_ns` | `robotdesc_ts_ns` |
| Planner 模块 | `soma_context.rs` | `robotdesc_context.rs` |
| Vitals 模块 | `soma_ingest.rs` | `robotdesc_ingest.rs` |
| mock 实现 | `mock_soma.rs` / `MockSomaService` | `mock_robotdesc.rs` / `MockRobotdescService` |
| endpoint 参数 | `--soma-endpoint` / `ROBONIX_SOMA_ENDPOINT` | `--robotdesc-endpoint` / `ROBONIX_ROBOTDESC_ENDPOINT` |
| stage pipe | `ROBONIX_SOMA_STAGE_FD` | `ROBONIX_ROBOTDESC_STAGE_FD` |
| Vitals mock 参数 | `--mock-soma-*` / `ROBONIX_VITALS_MOCK_SOMA_*` | `--mock-robotdesc-*` / `ROBONIX_VITALS_MOCK_ROBOTDESC_*` |

## 完整修改过程

1. 保存工作树基线到 `/tmp/robonix-robotdesc-migration/status-before.txt`、`worktree-before.patch` 和 `index-before.patch`，确认不覆盖既有迁移。
2. 先移动系统目录和两个 capability 源目录，再更新 Cargo workspace、package、library、binary 和 `Cargo.lock`。
3. 更新 Robotdesc 的 build.rs、配置解析、deployment、launcher、health、service、store、runtime monitor、入口和 README；默认 provider、namespace、contract、IDL 和错误/日志文案全部使用新身份。
4. 更新 Src/`rbnx` 启动链：内置 binary 映射、`system.robotdesc` manifest 注入、stage-1 waiter、stage-2 inherited pipe、端口检查、CLI 参数和子进程环境变量全部改为 Robotdesc。
5. 更新 Planner、Vitals、World、G1、Scout2、Webots、Piper、测试夹具、导航/地图调用方、CI、label、CODEOWNERS、README 和启动脚本。
6. 将示例本体描述文件从 `soma*.yaml` 改为 `robotdesc*.yaml`，并同步 manifest、环境变量和 Planner include 路径。
7. 更新健康 IDL 和所有 contract 生成输入；不手工修改生成 protobuf/gRPC 代码，使用构建和 codegen 重新生成。
8. 对活跃源码做旧名审计；历史 `CHANGELOG`、迁移记录、备份、缓存和生成物单独保留，不把历史文本误判为活跃组件身份。
9. 运行编译、测试、codegen、Python 回归、静态 AST 和真实 Src↔Robotdesc 注册验收；记录 Clippy、`git diff --check` 和 Conda/ROS Python 的限制。
10. 新增 `system/robotdesc/scripts/smoke_runtime.py`，实际走 `rbnx boot`、primitive INIT/ACTIVE、Robotdesc 五个 RPC、World footprint 调用和 stage 2 trigger，并检查退出清理。

## 关键运行链路

- `rbnx boot` 现在发现 `system.robotdesc` 时启动 `robonix-robotdesc`。
- Robotdesc 以 `provider_id=robotdesc` 注册 `robonix/system/robotdesc`，并声明 `footprint`、`get_health`、`get_urdf`、`get_yaml`、`health` 五个 gRPC capability。
- Robotdesc 仍先加载本体描述并启动 primitive stage 1，之后注册自身并进入 ACTIVE；rbnx 完成系统/service bring-up 后通过 `ROBONIX_ROBOTDESC_STAGE_FD` 发送 `stage2`，再启动 skill。
- Planner、Vitals、World 和 robot-description primitive 均通过新 contract id、generated service 类型和新 endpoint 连接，不再依赖旧 `soma` provider 或 namespace。

## 验证结果

已通过：

```text
cargo fmt --all -- --check
cargo check --workspace --all-targets
cargo test --workspace --all-targets: 272 passed, 0 failed（含新增 2 项迁移回归）
rbnx codegen -p system/world --mcp --clean --python /usr/bin/python3
World pytest: 244 passed
Python AST: 247 files parsed
Robotdesc boot smoke: primitive INIT/ACTIVE, five RPCs, World footprint, stage 2 trigger
```

真实进程验收使用 `/usr/bin/python3`、临时 manifest、临时 `robotdesc.yaml` 和临时 URDF，启动独立 Src 与 Robotdesc，验证结果：

```text
provider= robotdesc
namespace= robonix/system/robotdesc
capabilities= 5
contracts= robonix/system/robotdesc/footprint,
           robonix/system/robotdesc/get_health,
           robonix/system/robotdesc/get_urdf,
           robonix/system/robotdesc/get_yaml,
           robonix/system/robotdesc/health
old provider soma: absent
```

测试日志目录：`/tmp/robotdesc-migration-smoke-leAdRw/`。测试退出时仅停止本次创建的 Src/Robotdesc 进程，未停止用户已有服务。

跨进程启动 smoke 日志目录：`/tmp/robotdesc-boot-smoke-uq7iatc4/`。它验证 `rbnx boot` 使用新内置 binary、primitive 在 Robotdesc ACTIVE 之前通过注入的 endpoint 读取 URDF、随后进入 ACTIVE、Robotdesc 注册五个 contract、World 实际读取 footprint，以及 stage 2 trigger 写入。该 fixture 使用空 service/skill 列表，不等于 G1 完整部署验收。

## 旧名审计与保留项

- 活跃组件身份、manifest、contract、环境变量、运行脚本和 package 路径已使用 `robotdesc`；旧名仅保留在负向回归断言、历史 CHANGELOG、迁移记录、缓存/备份/历史生成物等范围。
- `scene` 在 World 的 `scene_graph`、语义对象和历史数据键中仍是既有业务语义；本轮不把它改成 Robotdesc，也不把 Robotdesc 与 World 混为一个组件。
- `robot_description` 是 primitive 的能力/包名称，表示 ROS robot_state_publisher 适配器；它通过 Robotdesc contract 获取 URDF，不等同于系统组件名。

## 未完成或受环境阻塞项

- `cargo clippy --workspace --all-targets -- -D warnings` 仍被工作树既有 `system/executor/src/service.rs` 的 4 个 nested-if / `or_insert_with(String::new)` 告警阻断；未为本轮迁移修改无关 Executor 控制逻辑。
- `git diff --check` 仍报告 G1 manifest 第 137、347 行尾随空格；该问题已在前一轮工作树中存在，未顺手清理无关格式。
- 使用 Conda Python 3.13 的 smoke 首次因 `grpcio 1.83.0` 与生成代码要求 `>=1.83.1` 失败；改用 `/usr/bin/python3`（ROS Humble，grpcio 1.83.1）后真实进程验收通过。后续必须保持 codegen、ROS 和运行时解释器 ABI 一致。
- 尚未执行真实 Unitree/Scout2/Webots 硬件、G1 完整 `rbnx boot`、Nav2、相机/VLM、健康 primitive 真实数据、Docker 构建及地图 Save/Load 验收。
- G1 本地 `rbnx-boot/cache/primitive-robot-description-rbnx` checkout 已临时同步 5 个源码/文档文件并重新运行 `rbnx codegen --clean --ros2 --python /usr/bin/python3`（72 个 Python 模块）。远程仍是 `f3b9f38`，没有提交或推送；缓存清理会丢失本地修改。这不是已完成的上游发布，必须在独立上游仓库审查、提交、发布后更新部署 revision。

## 可重复验收命令

```bash
cd /data/robonix
cargo fmt --all -- --check
cargo check --workspace --all-targets
cargo test --workspace --all-targets
rbnx codegen -p system/world --mcp --clean --python /usr/bin/python3
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 PYTHONDONTWRITEBYTECODE=1 \
PYTHONPATH="$PWD/system/world:$PWD/pylib/robonix-api:$PWD/system/world/rbnx-build/codegen/proto_gen:$PWD/system/world/rbnx-build/codegen/robonix_mcp_types" \
/usr/bin/python3 -m pytest system/world/tests -q
cargo clippy --workspace --all-targets -- -D warnings
```

Robotdesc 单进程注册验收需要准备临时 `robotdesc.yaml`、URDF 和空 primitive/skill manifest，然后启动：

```bash
robonix-src --listen 127.0.0.1:<SRC_PORT> --capabilities /data/robonix/capabilities
robonix-robotdesc --src 127.0.0.1:<SRC_PORT> \
  --listen 127.0.0.1:<ROBOTDESC_PORT> --provider-id robotdesc \
  --robot-yaml /tmp/robotdesc-smoke/robotdesc.yaml \
  --deployment-manifest /tmp/robotdesc-smoke/robonix_manifest.yaml
```

然后使用 `/usr/bin/python3` 加载 `$ROBONIX_SRC`，查询 provider `robotdesc`，检查 namespace 和五个 contract，并确认查询 `soma` 返回空结果。不要用未加载 ROS overlay 或 Conda 中 ABI 不匹配的解释器替代上述运行时。

运行可重复的跨进程 boot smoke（随机 loopback 端口和临时部署目录，无硬件动作）：

```bash
cargo build -p robonix-cli -p robonix-src -p robonix-robotdesc
/usr/bin/python3 system/robotdesc/scripts/smoke_runtime.py
```

验证后正式部署前运行 `make install`，确保 PATH 上的 `rbnx`、Planner、Vitals 与 `robonix-robotdesc` 来自同一套新源码；只重编译 rbnx 而继续使用旧系统二进制会产生接口不一致。

## 文档和提交状态

- 完整说明：`system/robotdesc/docs/soma-to-robotdesc-migration.md`。
- 完整路径清单：`system/robotdesc/docs/soma-to-robotdesc-files.txt`。
- 跨进程验收：`system/robotdesc/scripts/smoke_runtime.py`。
- 上游源码补丁：`system/robotdesc/docs/upstream-robot-description.patch`，在独立仓库审查并用 `git apply --check` 检查后使用；这不代表已经发布上游。
- 根 `docs/soma-to-robotdesc-migration.md` 是同步副本；根 `docs/` 当前为 Git 子模块，主仓库是否记录该副本取决于子模块状态。
- 本轮没有 `git commit`、`git push` 或创建分支。
