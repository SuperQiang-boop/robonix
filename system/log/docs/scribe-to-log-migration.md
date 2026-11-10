# Scribe → Log 命名迁移记录

## 目标与边界

记录日期：2026-11-09。本次把日志库组件 `Scribe` 的运行时身份迁移为 `Log`：系统目录、Rust crate/binary 名、库名、Python 门面模块、环境变量、`rbnx` 集成、CI/维护者表与文档同步改名。

`Scribe` 不是可启动的系统进程：它没有 IDL、contract、provider 或 `rbnx boot` 内置 binary，因此本轮**不涉及 wire 协议、gRPC 服务名或 provider namespace**，只改库身份、日志目录/级别环境变量和 Python 门面。日志记录的字段、格式、级别语义和文件布局保持不变。

迁移不改变 `services/memory` 的产品名 “Scribe Mem” 与其消费的 “Scribe Log” 格式名（README 中已标注 formerly Scribe Mem），也不改该服务自带的 `memory_service/scribe_log.py`；这些是历史产品/格式专有名，不属于日志组件身份。除 `ROBONIX_LOG_DIR` 这一跨进程环境变量外，memory 的内部命名不动。

工作树已包含 Atlas→Src、Pilot→Planner、Liaison→Gateway、Scene→World、Soma→Robotdesc、Vitals→Monitor 的未提交改动（基线 HEAD `f6850dcd`，分支 `rename`）。本轮未回滚、未清理、未提交、未创建分支。

## 名称映射

| 范围 | 旧名 | 新名 |
|---|---|---|
| 系统目录 | `system/scribe` | `system/log` |
| Cargo package | `robonix-scribe` | `robonix-log` |
| Rust library | `robonix_scribe` | `robonix_log` |
| 组件文档名 | `Scribe` | `Log` |
| Python 门面 | `robonix_api/scribe_logger.py` | `robonix_api/log_logger.py` |
| 日志目录环境变量 | `SCRIBE_LOG_DIR` | `ROBONIX_LOG_DIR` |
| 控制台级别环境变量 | `SCRIBE_CONSOLE_LEVEL` | `ROBONIX_LOG_CONSOLE_LEVEL` |
| 文件级别环境变量 | `SCRIBE_FILE_LEVEL` | `ROBONIX_LOG_FILE_LEVEL` |
| 脚本中的旧级别变量 | `SCRIBE_STDOUT_LEVEL` | `ROBONIX_LOG_STDOUT_LEVEL` |
| CI 标签 | `comp:scribe` | `comp:log` |
| 维护者表路径 | `system/scribe` | `system/log` |

## 修改范围

- **Rust 库**：`system/scribe/` → `system/log/`（`Cargo.toml`、`README.md`、`src/{lib,format,sink}.rs`）；crate/lib 名、文档注释、`$ROBONIX_LOG_DIR` 说明与内部日志级别辅助函数注释同步。
- **Workspace 与依赖**：根 `Cargo.toml` 的 member `system/log` 与 `robonix-log` workspace 依赖、`Cargo.lock`，以及 `system/{src,executor,gateway,planner,robotdesc,monitor}`、`tools/{rbnx,codegen}` 的 `robonix-log.workspace = true` 依赖和全部 `robonix_scribe::` 调用。
- **rbnx**：`ROBONIX_LOG_*` 环境变量写入/读取、`rbnx logs` 的默认目录与帮助文案、`rbnx boot` 对子进程日志目录的注入、`--log` 配置说明，以及 deploy/run_package/process/database/manifest 中的 log 门面调用；`use robonix_scribe as scribe` 改为直接 `use robonix_log as log`。
- **Python**：`robonix-api` 的 `scribe_logger` → `log_logger`（模块、`__init__` 导出、`capability.py` 自动初始化），Speech/Memsearch/Voiceprint 服务的 import 与 `install_stdlib_bridge` 调用；memory 服务只把 `SCRIBE_LOG_DIR` 读取改为 `ROBONIX_LOG_DIR`。
- **脚本与示例**：`scripts/test_*` 的级别环境变量、`system/world/scripts/smoke_runtime.py` 日志目录、G1/Scout2/Agilex 示例中提及 log 门面的注释。
- **治理与文档**：`.github/{CODEOWNERS,labeler.yml,labels.yml,workflows/issue-triage.yml,workflows/testing.yml}`、issue 模板、`README.md`、`MAINTAINERS.md`、相关组件 README。
- 逐文件清单见 `system/log/docs/scribe-to-log-files.txt`（6 条 move、80 条 update）。

## 执行过程

1. 保存基线 `git status --short`（755 条），用词边界搜索 `\b[Ss]cribe\b|SCRIBE_|robonix[-_]scribe|scribe_logger` 过滤掉 describe/subscribe/inscribed 等英文噪声，得到 85 个候选文件。
2. 先迁目录与 Python 门面文件：`system/scribe` → `system/log`，`scribe_logger.py` → `log_logger.py`。
3. 按映射逐文件替换 crate/库/类型/环境变量/文档：`Scribe` → `Log`、`scribe` → `log`、`SCRIBE_*` → `ROBONIX_LOG_*`；不含 `services/memory` 的产品名与格式名。
4. 手工修正机械替换产生的别扭文案（如 “log log-file floor”“into log via”“log logs”），并让 memory 服务改读 `ROBONIX_LOG_DIR`。
5. `cargo fmt --all`、`cargo check --workspace --all-targets`、`cargo test -p robonix-log`、`cargo test --workspace --all-targets`。
6. 用隔离目录验证 Rust 与 Python 两端都写入 `ROBONIX_LOG_DIR`，并用 `rbnx logs` 回读。
7. 旧名审计并记录保留项。

## 关键运行链路

- 每个 Rust 组件启动时调用 `robonix_log::init_from_config(tag, config_json)`；`init_from_config` 读取 manifest 的 `log` 字段设置文件 sink 级别，未设置时回退到 `ROBONIX_LOG_FILE_LEVEL` / `Info`。
- 文件 sink 写入 `$ROBONIX_LOG_DIR`（未设置时 `./logs`），按 tag 生成 `<tag>.log`；`rbnx boot` 为子进程注入 `ROBONIX_LOG_DIR`，`rbnx logs` 默认读取 `<manifest-dir>/rbnx-boot/logs` 或 `$ROBONIX_LOG_DIR`。
- Python 服务 `from robonix_api import log_logger` 后调用 `install_stdlib_bridge(tag)`，与 Rust 端共享同一目录和 JSON-lines 格式。
- `services/memory` 继续使用自己的 `scribe_log.py`（Scribe 格式）与 “Scribe Mem” 文案，但日志目录改为读取 `ROBONIX_LOG_DIR`，保证与 rbnx 注入一致。

## 验证结果

```text
cargo fmt --all                             通过
cargo check --workspace --all-targets       通过
cargo test -p robonix-log                   26 passed（另有 2 个 ignored doctest）
cargo test --workspace --all-targets        272 passed, 0 failed
cargo clippy -p robonix-log --all-targets -- -D warnings   通过
cargo clippy --workspace --all-targets -- -D warnings      仍被 system/executor/src/service.rs 的 4 个既有告警阻断
```

隔离环境验证（`ROBONIX_LOG_DIR=$PWD/.log-smoke`）：

```text
Python:  log_logger.info('py_smoke', ...)      → .log-smoke/py_smoke.log 写入 2 条 JSON-lines
Rust:    robonix-src --log info                → .log-smoke/src.log 写入启动与 contract registry 记录
rbnx:    rbnx logs -d .log-smoke -t py_smoke   → 正确渲染 logcat 风格输出
rbnx:    rbnx logs --help                      → 默认目录说明为 $ROBONIX_LOG_DIR
```

## 旧名审计与保留项

- 活跃 Rust/Python/脚本/CI 中已无 `robonix-scribe`、`robonix_scribe`、`SCRIBE_*`、`scribe_logger`。
- 保留：`services/memory` 的 “Scribe Mem” 产品名、“Scribe Log” 格式名、`memory_service/scribe_log.py` 及其类/函数名（按边界决定不随组件改名）；`examples/webots/robonix_manifest.yaml` 的 “Memgraph (Scribe Mem)” 注释。
- 历史与生成物：`CHANGELOG.md`、`docs/*-migration.md`、`system/{world,robotdesc}/docs/*`；未跟踪的 `examples/robot-unitree-g1/rbnx-boot/processes.json`（旧 boot 运行时状态）和 `pylib/robonix-api/robonix_api.egg-info/SOURCES.txt`（打包元数据）仍含旧名，属构建/运行产物，未手改。
- `SCRIBE_STDOUT_LEVEL` 在迁移前就不被库读取（库只读 `SCRIBE_CONSOLE_LEVEL`）；本轮按字面改名为 `ROBONIX_LOG_STDOUT_LEVEL`，保持既有（不生效）行为，未顺手改语义。
- `.github/workflows/issue-triage.yml` 的关键词 `scribe` 改为 `log` 后语义过宽，可能对标题含 “log” 的 issue 误加 `comp:log` 标签；本轮只做改名，是否需要收窄匹配留待 CI 维护者决定。

## 未完成或受环境阻塞项

- workspace clippy 仍被 `system/executor/src/service.rs:220/242/252/253` 的 4 个既有告警阻断，非本轮引入。
- 未运行完整 `rbnx boot` 部署（Webots/G1/Scout2）与 Docker 构建；本轮只验证了库、Python 门面、`rbnx logs` 与 `rbnx boot` 的日志目录注入路径。
- 迁移过程中 clippy 曾因磁盘写满中断；已清理 `target/debug/incremental` 与旧 crate 名构建目录后重跑，clippy 结果以上述既有告警为准。
- 全部改动未提交、未推送；`docs/` 为未初始化子模块，根目录记录副本不被主仓库跟踪，主记录在 `system/log/docs/`。

## 可重复验收命令

```bash
cd /data/robonix
cargo fmt --all -- --check
cargo check --workspace --all-targets
cargo test -p robonix-log
cargo test --workspace --all-targets
cargo clippy -p robonix-log --all-targets -- -D warnings

# 环境变量与两端日志写入
mkdir -p /tmp/log-smoke
ROBONIX_LOG_DIR=/tmp/log-smoke PYTHONPATH="$PWD/pylib/robonix-api" \
  /usr/bin/python3 -c "from robonix_api import log_logger; log_logger.info('smoke','ok')"
ROBONIX_LOG_DIR=/tmp/log-smoke ./target/debug/robonix-src \
  --listen 127.0.0.1:15351 --capabilities "$PWD/capabilities" --log info
./target/debug/rbnx logs -d /tmp/log-smoke
```

## 文档与提交状态

- 本记录：`system/log/docs/scribe-to-log-migration.md`，根 `docs/` 下为同步副本。
- 逐文件映射：`system/log/docs/scribe-to-log-files.txt`。
- 本轮没有 `git commit`、`git push` 或创建分支。
