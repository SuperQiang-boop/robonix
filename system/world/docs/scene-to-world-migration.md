# Scene → World 命名迁移记录

## 目标与边界

本次迁移把系统组件 `Scene` 的身份统一为 `World`，不改变能力的业务语义、消息字段、ROS topic、地图数据格式或运行时行为。`scene graph`、`SceneGraphNode`、`scene.object.*`、`scene_maps`、`scene_memory` 等仍表示语义地图、对象标识或持久化数据格式，不是系统组件身份，因此保留。

本记录对应 2026 年 11 月 6 日工作树中的迁移。工作区同时包含此前 Atlas→Src、Pilot→Planner、Liaison→Gateway 的未提交修改；后续操作不得用清理或回滚覆盖这些改动。

## 名称映射

| 范围 | 旧名 | 新名 |
|---|---|---|
| 系统目录 | `system/scene` | `system/world` |
| 系统 capability 目录 | `capabilities/system/scene` | `capabilities/system/world` |
| Python 包 | `robonix-system-scene` | `robonix-system-world` |
| Python 入口 | `scene_service` | `world_service` |
| package | `com.robonix.system.scene` | `com.robonix.system.world` |
| provider | `Service(id="scene")` | `Service(id="world")` |
| capability namespace | `robonix/system/scene` | `robonix/system/world` |
| Docker 镜像 | `robonix-scene` | `robonix-world` |
| Docker 容器 | `robonix_scene` | `robonix_world`（`ROBONIX_WORLD_CONTAINER` 可覆盖） |
| 环境变量 | `SCENE_*`、`ROBONIX_SCENE_*` | `WORLD_*`、`ROBONIX_WORLD_*` |
| 生成目录 | `scene_proto_gen` | `world_proto_gen` |
| 依赖文件 | `scene-*.txt` | `world-*.txt` |
| 验证脚本 | `verify_scene_map_persistence.py` | `verify_world_map_persistence.py` |
| CI 组件标签 | `comp:scene` | `comp:world` |

Planner、G1、Scout2、Webots、CI、Docker、启动脚本、manifest、protobuf/MCP 运行时目录和测试调用端均使用新系统身份。World 连接 Soma 时的 `consumer_id` 也必须是 `world`；否则 Src 注册和下游审计仍会显示旧身份。

## 完整修改范围

1. 阅读根 `AGENTS.md`、适用接入技能和已有迁移记录；保存 `git status --short`，保护工作区中其它迁移。
2. 逐文件搜索目录名、crate/package/binary、provider id、namespace、contract id、环境变量、manifest、脚本、Docker、生成目录、测试和文档。
3. 先迁移目录和 capability 源文件，再更新 Python/Rust 包元数据、workspace、启动入口和调用方。不要直接编辑生成的 protobuf、gRPC、MCP 或 reference 文档。
4. 更新 World 的 `package_manifest.yaml`、`pyproject.toml`、`README.md`、`scripts/build.sh`、`scripts/start.sh`、`scripts/start_native.sh`、Dockerfile 和 entrypoint；确认启动脚本设置新生成目录及 PYTHONPATH。
5. 更新 Src 注册、Planner discovery、World→Soma 连接、examples、CI labeler/triage、测试夹具和测试断言。
6. 重新生成 World 的源代码：

   ```bash
   rbnx codegen -p system/world --mcp --clean --python /usr/bin/python3
   ```

   生成物位于 `system/world/rbnx-build/`，属于构建产物，不作为手工迁移输入。

7. 对活跃源码审计旧身份；排除 `.git`、`target`、`rbnx-build`、缓存、备份、历史 CHANGELOG 和迁移记录。剩余命中必须逐项判定：系统身份改为 World，语义地图词保留。
8. 运行目标测试、代码生成验证、Rust 检查和跨进程 boot/smoke。命名跨越 Src、Driver、MCP、ROS 或 Docker 时，不能只以编译通过作为验收。
9. 记录真实硬件/模型未执行的部分；不提交、不建分支、不清理无关改动，除非用户另行要求。

## 语义名称保留规则

- 保留 `scene graph`、`SceneGraphNode`、`get_scene_graph` 和 `scene.object.*`，因为它们是语义地图 contract、类型或对象 ID。
- 保留 `scene_maps`、`scene_memory`、`scene_annotations` 等已有数据目录或持久化键，避免破坏已有数据和回滚读取。
- Three.js 的 `THREE.Scene`、页面 DOM 的内部变量，以及历史测试中的语义对象名不属于系统组件身份，不做机械改名。
- 日志前缀、错误提示、consumer id、provider id、容器/包/namespace、UI 中指代系统组件的文案统一使用 World。

## 文件清单（按职责）

- **系统实现**：`system/world/` 下的 package manifest、Python 包、Docker、build/start/stop 脚本、World README 和测试。
- **能力与 IDL**：`capabilities/system/world/`，以及引用 `robonix/system/world` 的 manifest、Planner contract 和生成输入。
- **运行时调用方**：`system/planner`、`system/src`、`system/executor`、`system/soma`、`tools/rbnx`、G1/Scout2/Webots/piper 示例。
- **部署与质量**：`.github/CODEOWNERS`、`.github/labeler.yml`、`.github/labels.yml`、issue triage、CI workflow、Docker 配置、testing README 和验证脚本。
- **记录**：本文件以及既有 `atlas-to-src`、`pilot-to-planner`、`liaison-to-gateway` 迁移记录。

逐文件完整路径见主仓库中的 `system/world/docs/scene-to-world-files.txt`：包含原目录全部 84 个文件的旧→新映射、8 个 capability 文件及调用方清单。备份文件只随目录搬迁，不做内容替换；列表中的调用方可能同时带有此前其它命名迁移的改动，并不表示这些 diff 全由本轮产生。

## 验证结果

已通过：

```text
rbnx codegen -p system/world --mcp --python /usr/bin/python3
首次增量生成验证 76 个 Python 模块；--clean 重新生成后验证 72 个模块
World pytest: 244 passed
cargo fmt --all -- --check: 通过
cargo check --workspace --all-targets: 通过
cargo test --workspace --all-targets: 通过（270 tests）
bash -n: World 的 build/start/native/entrypoint 通过
World source/tests/scripts: AST 解析通过
```

本轮新增并通过 `system/world/scripts/smoke_runtime.py`：启动真实隔离 Src 与 World 进程，通过独立 ROS domain 的 reliable/transient-local 发布器输入 4×3 OccupancyGrid，验证 web API 返回 PNG、HTTP 创建房间后通过真实 MCP `list_regions` 读回同名房间、8 个工具及 namespace 注册，并确认旧 `scene` provider/namespace 不存在。World 正常退出码为 0，测试所启动的 Src 收 SIGTERM 后退出，未遗留进程。成功日志目录：`/tmp/world-migration-smoke-fnw2i_na/`。该验收关闭 object persistence 和 VLM/重模型，不能替代相机、Soma 真机、导航、地图 Save/Load、完整 boot 或 Docker 验收。

前期测试曾发现相机夹具仍注入 `scene` 而运行函数使用 `world`，以及页面断言仍期待 Scene 文案；已同步。图像关系测试原来依赖隐式 event loop，在其它测试 `asyncio.run()` 后会失败；改为显式创建/关闭循环或直接 `asyncio.run()`，不修改生产算法。补充了 Soma footprint 调用必须以 `consumer_id="world"` 连接并关闭 channel 的回归测试。

真实 Docker 构建已执行 `rbnx build -p system/world --no-update-check`，最终失败：concept-graphs 镜像站 Git TLS 失败，回退 GitHub 后连接 443 超时，Docker 构建层退出 128、rbnx 退出 1，因此新镜像未完成。构建日志另出现 `grpcio-tools 1.76.0` 要求 `protobuf<7` 而后续层安装 `protobuf 7.36.2` 的冲突告警。日志：`/tmp/world-rbnx-build.log`。这些依赖/网络问题独立于改名，不通过加旧名 alias 或随意放宽依赖修复；网络恢复后重试构建，并复核 protobuf 工具链兼容性后再验收镜像。没有停止或重启用户已有服务。

当前 `cargo clippy --workspace --all-targets -- -D warnings` 被工作树中 `system/executor/src/service.rs:220`、`:242`、`:252`、`:253` 的 4 个告警阻断（可合并的 nested `if` 和 `or_insert_with(String::new)`）；未扩大本次命名迁移去修改 Executor 的控制逻辑或放宽 lint。仓库整体不能宣称 clippy 全绿。`git diff --check` 另报告 G1 manifest 第 137、347 行原有空白行尾空格，未清理无关配置。

## 部署验收

```bash
rbnx build -f examples/robot-unitree-g1/robonix_manifest.yaml --no-update-check
rbnx boot -f examples/robot-unitree-g1/robonix_manifest.yaml --no-update-check
rbnx caps -v
```

应观察到：

- system 阶段显示 `world`，而不是 `scene`；
- World provider 在 `robonix/system/world` 注册；
- Planner 发现 `list_objects`、`get_robot_context`、`get_scene_graph` 等 contract；
- World 容器启动后只使用 `WORLD_*` / `ROBONIX_WORLD_*` 配置；
- 出现失败时先查看 World 日志、Src capability 查询和生成目录，而不是直接改缓存。

## 排障与回滚

1. `no package_manifest.yaml`：检查目录是否为 `system/world`，manifest 是否仍引用旧路径。
2. `contract not found`：检查 `capabilities/system/world/`、manifest contract id、Src capabilities 根路径和 `rbnx codegen` 输出。
3. `world` 能启动但 Planner 找不到：检查 `Service(id="world")`、namespace、provider id 以及 Planner 的 discovery filter。
4. World 能启动但没有相机/地图：检查 `config.observations[]`、ROS2 overlay、Src `find_capability` 结果和 `WORLD_*` 配置；不要把 `scene graph` 语义名称改成新的 contract。
5. 回滚时恢复系统目录、package/provider/namespace、manifest 和启动脚本的身份映射；保留语义数据目录和 `scene.object.*` 数据键，避免把数据格式迁移误当成组件回滚。

## 文档保存和上游注意事项

当前根 `docs` 在 Git 中是未初始化的子模块，`docs/src/developer-guide.md` 不在本机。不能把在这个目录写出的文件误认为主仓库已跟踪，因此本记录也保存为主仓库可跟踪的 `system/world/docs/scene-to-world-migration.md`；不擅自初始化、覆盖或提交文档子模块。恢复子模块前应先备份根 `docs/` 下既有迁移记录。

本轮没有提交、推送或修改远程。`mapping_rbnx`、`explore_rbnx` 等独立上游中的改名必须在它们的源码仓库单独审查、提交并发布，再更新部署；不得只改 `rbnx-boot/cache` 并宣称上游迁移完成。

## 可重复验收命令

```bash
cd /data/robonix
rbnx codegen -p system/world --mcp --clean --python /usr/bin/python3
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 PYTHONDONTWRITEBYTECODE=1 \
PYTHONPATH="$PWD/system/world:$PWD/pylib/robonix-api:$PWD/system/world/rbnx-build/codegen/proto_gen:$PWD/system/world/rbnx-build/codegen/robonix_mcp_types" \
  /usr/bin/python3 -m pytest system/world/tests -q
cargo fmt --all -- --check
cargo clippy --workspace --all-targets -- -D warnings
cargo test --workspace --all-targets
cargo build -p robonix-src
source /opt/ros/humble/setup.bash
/usr/bin/python3 system/world/scripts/smoke_runtime.py
```

pytest 禁用插件自动加载是为了绕开宿主用户目录 anyio 插件与系统旧 pytest 的版本冲突，不等同于关闭测试。不要重新进入 conda 后直接用非 ROS ABI 匹配的 Python 替代 `/usr/bin/python3`。

## 修改过程中的修正与经验

| 环节 | 发现的问题或约束 | 处理与复用要求 |
|---|---|---|
| 组件身份 | 只改目录不能覆盖注册身份、调用者身份及发现过滤条件 | 同步 package、provider、namespace、manifest、Planner discovery，以及 World→Soma 的 `consumer_id="world"` |
| 运行入口 | Python 模块、Docker 镜像/容器、环境变量及 codegen 目录属于不同入口 | 分别核对 build、start、native、entrypoint 和依赖文件，不用旧名称 alias 掩盖遗漏 |
| 源码与生成物 | 增量生成时验证了 76 个 Python 模块，清理重生后为 72 个 | 最终以 `--clean` 重生结果为准；不要手工替换生成代码 |
| 回归测试 | 相机测试夹具仍注入 `scene`，页面断言仍期待 Scene 文案 | 按新组件身份同步夹具和断言，保留真正表达语义地图的 `scene` 名称 |
| 测试隔离 | 图像关系测试依赖隐式 event loop，受其它测试 `asyncio.run()` 影响 | 显式管理测试循环，不为测试顺序修改生产算法 |
| 跨进程验证 | 单测通过不能证明 Src 注册、ROS 数据和 MCP 调用链正常 | 新增隔离 smoke，验证真实进程、OccupancyGrid、PNG、房间读回和旧身份不存在 |
| 文档归档 | 根 `docs/` 是未初始化子模块，根目录副本不属于主仓库普通文件 | 主记录和逐文件清单放在 `system/world/docs/`；保留根副本但不宣称其已被 Git 跟踪 |
| 部署验收 | Docker 下载失败、protobuf 冲突告警，且还有既有 lint 问题 | 分别记录网络、依赖和 lint 阻塞，不把源码迁移通过等同于完整部署通过 |

## 记录复核（2026-11-07）

- 清单 `scene-to-world-files.txt` 共 **165 条记录**：92 条 `move`（84 个原组件文件及 8 个 capability 文件）、70 条 `update`、3 条 `add`；逐条目标文件均存在，无重复目标路径。
- 清单按文件路径归档，不表示每个文件的所有未提交 diff 都由本轮迁移产生；备份文件只计目录搬迁，生成物、运行日志和根 `docs/` 副本不计入上述记录数。
- 复核 `/tmp/world-cargo-unit.log`：此前后台 Rust 测试已结束，17 个结果段合计 **270 passed、0 failed**；未发现仍在运行的该 workspace 测试进程。这是对既有验收日志的核对，本次文档补录没有重跑全部构建和测试。
- 根 `docs/scene-to-world-migration.md` 与本文件保持内容一致；主仓库中的文档仍是未提交文件，不代表已提交或已发布。
- 尚待处理：恢复 concept-graphs 下载并复核 protobuf 兼容性后重试 Docker 构建；处理既有 clippy/空白问题后重新检查；完成真实硬件、完整 `rbnx boot`、相机/VLM、Nav2 与地图 Save/Load 验收。

下次复用本流程时，先保存当时的工作树基线，再逐文件建立旧→新映射，并为每项记录实际修改及验证结果。最终交付须同时包含迁移说明、完整路径清单、可重复命令和未完成项；只有上述真实部署验收完成后，才能将部署状态改为通过。
