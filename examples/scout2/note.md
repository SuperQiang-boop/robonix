# 地图与物体信息持久化

## 快速索引

- [切换 Linux 用户后的 Python 环境与 Scout2 启动](#切换-linux-用户后的-python-环境与-scout2-启动)
- [停机后手动清理全部地图与 Scene 数据](#停机后手动清理全部地图与-scene-数据)

## 修改目标

地图重启后可以从磁盘加载，但地图上的椅子、桌子等 Scene 物体并不属于地图栅格或 RTAB-Map 数据库；如果只保存地图文件，物体坐标、包围盒和语义属性不会随地图恢复。本次修改的目标是让建图期间识别到的物体坐标持续写入持久化存储，重启后按地图恢复，并提供 REST 接口查询指定地图的物体列表。

## 实现过程

1. Scout2 部署配置启用 `SCENE_RESTORE_ON_START: "true"`，Scene 启动时将对象状态绑定到配置的地图 ID `scout2_main`。
2. Scene 后台任务默认每 3 秒把当前地图中非机器人对象的位置写入对象数据库。该间隔可由 `SCENE_OBJECT_PERSIST_INTERVAL_S` 调整。地图切换或坐标系世代变化时，Scene 清理旧活动分区，避免把旧地图坐标误认为新地图坐标。
3. Scene 容器通过持久化数据目录挂载保存对象数据库，因此容器退出或项目重启后数据库仍在。显式保存地图时，Scene 也会把物体快照写入独立分区，并将该分区记录在地图元数据中，供加载已保存地图时恢复。
4. 新增 `GET /api/maps/{map_id}/objects`，按地图 ID 查询对象坐标等信息。

## 文件位置及地图关联

地图与物体使用不同存储，靠相同的 `map_id` 关联：

| 数据 | 默认位置 | 内容 |
| --- | --- | --- |
| 地图 | `services/service-map-rbnx/maps/<map_id>/`，Scout2 示例为 `services/service-map-rbnx/maps/scout2_main/` | RTAB-Map 地图数据库 `rtabmap.db` 及地图预览、元数据等文件 |
| Scene 物体 | `system/scene/rbnx-build/data/robonix/scene_memory/objects.db` | SQLite 对象记录；活动记录按地图 ID 分区，显式地图快照使用独立分区 |

Scene 启动脚本将主机上的 `system/scene/rbnx-build/data/robonix/` 挂载为容器内 `/data/robonix/`。因此对象库**不放在** `maps/` 目录中。Scene 数据根目录可通过 `SCENE_DATA_DIR` 覆盖，对象库文件可通过 `SCENE_OBJECT_MEMORY_DB` 覆盖；地图目录可通过 `MAPPING_MAPS_DIR` 覆盖。部署时应保证这些目录位于持久化存储上。

地图 ID 是两类数据的关联键。Scout2 当前使用 `scout2_main`；查询该地图时，Scene 从对象库选择相应活动分区，或在活动分区为空时使用地图元数据指定的快照分区。加载已保存地图后，Scene 根据地图元数据恢复对应的对象快照。地图文件和对象库需要一起保留，才能跨重启恢复完整的地图与物体信息。

## REST 查询

请求示例：

```http
GET /api/maps/scout2_main/objects
```

成功响应包含 `map_id`、`generation`、`objects` 和 `relations`。每个对象当前返回 `id`、`class`、`pose`（`x`、`y`、`z`、`yaw`）、`frame_id`、`bbox`、`confidence`、`observation_count`、`last_seen` 和 `attributes`。若对象存储不可用，接口返回 `503`；指定地图没有可用物体分区时返回 `404`。

目前 `relations` 从当前活动 Scene 场景图读取；查询非活动历史地图时该字段可能为空，不应据此判断对象不存在。物体列表本身从该地图的持久化对象分区读取。

## 验证记录

本次相关代码已执行 Python 语法检查、启动脚本 `bash -n` 检查及 `git diff --check`。Scene 版本隔离、快照关联和删除清理测试已通过：
`PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest -q system/scene/tests/test_map_epoch.py system/scene/tests/test_persistence.py system/scene/tests/test_annotations.py`
结果为 `25 passed, 24 skipped`。跳过项依赖未安装的 Milvus 运行时。

## 同名地图重建时的物体错配与删除方案

  ### 问题与原因

  已实测：正常重启后，`scout2_main` 地图及其物体可以恢复，物体能够显示在地图上。但 `scout2_main` 是固定地图 ID；如果删除、重置或重新开始建图后仍使用该
  ID，旧物体记录可能被当作新地图的物体恢复，导致坐标错配。

  Mapping 默认每 30 秒保存地图检查点，Scene 默认每 3 秒保存物体，两者独立写入。突然断电后，不能仅凭相同的 `map_id` 判断物体坐标与实际加载的地图一致。

  ### 删除入口分析

  8091 页面的 `POST /api/delete` 调用 Mapping 的删除逻辑，只处理地图文件；不能用它单独完成“地图及物体一起删除”。Scene 已有 `POST /api/maps/delete`，其
  代码包含地图、对象分区及关联元数据的清理。使用前仍须验证运行中地图的删除限制，以及部分删除失败时的响应；不要在建图运行中直接删除当前地图。

  ### 解决方案（已实现）

  1. 给每次建图及其地图检查点保存可持久化的版本标识；物体记录关联同一版本。重启时校验实际加载地图与物体所属版本，不一致则禁止投影旧物体。
  2. 完善统一删除入口，对指定地图的地图文件、物体记录和关联元数据执行清理，并分别报告成功或失败。删除后重新使用 `scout2_main` 时，旧物体不得恢复。
  3. 验收正常重启、断电后从同一地图续建、同名地图删除后重建三种情况；同时检查接口中的物体 ID、`map` 坐标和 `/2d` 页面上的实际位置。

  以上版本校验、统一删除和失败保留重试逻辑已在 Scene 地图门面中实现，并有单元测试覆盖。当前仍未完成真实容器环境下的三类端到端验收：正常重启、断电续建、同名地图删除后重建；原因是当前环境缺少 `starlette`/部分运行时依赖且 Docker socket 不可用。

## 删除地图及对应物体

统一删除接口为 `POST /api/maps/delete`。请求体必须包含地图 ID，例如：

```bash
curl -X POST http://localhost:8091/api/maps/delete \
  -H 'Content-Type: application/json' \
  -d '{"map_id":"scout2_main"}'
```

该接口会调用 Mapping 删除地图文件，并清理 Scene 的对象持久化分区、地图元数据和关联场景标注。成功响应中的 `scene_cleanup.objects_deleted` 表示删除的物体记录数，`annotations_deleted` 表示标注是否删除。

如果部分清理失败，响应会包含 `scene_cleanup_error`；对象快照删除失败时会保留地图元数据，便于再次调用接口重试。8091 页面原有的 `POST /api/delete` 只处理地图文件，不能单独用于删除地图对应的 Scene 物体。

现已将 8091 的 `POST /api/delete` 接口改为联动删除：Mapping 成功删除地图文件后，会向 Scene 的 `POST /api/maps/delete` 发送同一个 `map_id`，清理对象、元数据和标注。Scene 地址由 `SCENE_WEB_URL` 配置，默认值为 `http://127.0.0.1:50107`。如果 Scene 未启动或调用失败，8091 仍返回地图删除结果，并在 `scene_cleanup_error` 中报告物体清理失败，便于重试 Scene 删除。

### 8091 联动删除实现

实现文件：`services/service-map-rbnx/src/mapping_rbnx/webui.py`。

调用 8091 的删除接口时，Mapping 先删除地图目录；删除成功后自动请求 Scene 的 `/api/maps/delete`。因此调用方式保持不变：

```bash
curl -X POST http://localhost:8091/api/delete \
  -H 'Content-Type: application/json' \
  -d '{"map_id":"scout2_main"}'
```

Scene 服务地址可通过 `SCENE_WEB_URL` 覆盖，默认是 `http://127.0.0.1:50107`。若 Scene 删除失败，8091 响应仍会返回 Mapping 的地图删除结果，并增加 `scene_cleanup_error` 字段。修改后需要重新构建并重启 Mapping 服务，才能让运行中的 8091 使用新逻辑。

## 删除正在加载的地图

当前实现不会直接删除正在使用的活动地图。`map_ops.delete_map_impl()` 会检查当前活动 `map_id`；如果目标地图正在加载，返回 `ok: false`，提示 `cannot delete the active map; load another map first`。由于 Mapping 删除失败，8091 也不会调用 Scene 删除该地图的物体信息。

推荐流程：

1. 先通过 `POST /api/load` 加载另一张地图，或切换到临时地图，使目标地图不再是活动地图。
2. 确认活动地图已切换。
3. 调用 8091 的 `POST /api/delete` 删除目标地图。Mapping 成功后会自动调用 Scene 的 `POST /api/maps/delete`，清理该地图的物体、元数据和标注。

如果系统只有这一张地图，不要直接删除 `maps/<map_id>/` 目录或手工修改对象数据库。应先停止相关服务并使用离线清理流程，避免 Mapping 仍持有已删除数据库、Scene 仍绑定旧坐标系，以及地图文件和物体记录状态不一致。当前 `POST /api/maps/delete` 也不会绕过活动地图保护。

## 切换 Linux 用户后的 Python 环境与 Scout2 启动

Scout2 原有部分 primitive 和 skill 启动脚本写死了旧用户目录，例如
`/home/user/miniforge3`、`/home/szh/miniforge3` 和
`/home/user/robonix`。切换到 `root`、`test` 或其他用户后，这些目录可能
不存在，或其中的 Python 环境不可访问，导致 primitive 在注册 Atlas 前
退出或超时。

当前脚本不再依赖旧用户的绝对路径，Python 运行环境按以下顺序选择：

1. 如果已经激活 Python 虚拟环境，即存在 `VIRTUAL_ENV`，使用
   `$VIRTUAL_ENV/bin/python3`。
2. 否则通过 `CONDA_EXE`、PATH、当前用户的 `$HOME` 以及 `/opt` 查找
   Conda，然后使用名为 `robonix-scout2` 的 Conda 环境。
3. Robonix Python API 通过 `rbnx path robonix-api` 定位当前源码目录；如需
   显式覆盖，可设置 `ROBONIX_API` 为包含 `robonix_api` 包的目录。

### 新用户推荐启动方式

不要复用 root 或其他旧用户的 Conda/虚拟环境。以新用户 `test` 为例，
先确认该用户能够读取仓库、写入 Scout2 的运行目录，并能找到 `rbnx` 和
`uv`：

```bash
cd /data/robonix
command -v rbnx
command -v uv
rbnx setup

uv venv --python /usr/bin/python3 --system-site-packages ~/.venvs/robonix-scout2
uv pip install --python ~/.venvs/robonix-scout2/bin/python \
  -e /data/robonix/pylib/robonix-api 'numpy<2' Pillow

cd /data/robonix/examples/scout2
source ~/.venvs/robonix-scout2/bin/activate
bash scripts/check_python_env.sh
rbnx boot
```

`--system-site-packages` 用于复用主机上 ROS 2 Humble 的 Python 包。每次
打开新终端后，都需要重新执行 `source`，再运行 `rbnx boot`。如果不激活
虚拟环境，脚本会回退到 Conda 查找；新用户必须自行安装或配置 Conda，
并创建、安装好 `robonix-scout2` 环境，不能依赖其他用户家目录中的环境。

### 权限与运行前检查

- `rbnx setup` 按用户保存 Robonix 源码路径，新用户需要单独执行。
- 新用户需要能够执行 `rbnx`；若二进制只安装在 `/root/.cargo/bin`，应为
  新用户安装，或把一个所有目标用户可执行的位置加入 PATH。
- 新用户需要对 `examples/scout2/rbnx-boot/` 及相关构建、日志目录拥有写
  权限，并拥有相机、雷达、CAN 等设备所需的系统权限。
- `bash scripts/check_python_env.sh` 会检查 ROS、MCP、protobuf 生成模块和
  gRPC 环境，但不会启动机器人硬件。

### 当前验证结论

在 `/data/robonix/examples/scout2/.venv` 激活后，Python 环境检查通过；
`audio_driver`、`robot_description` 和 `scout2_chassis` 在真实启动中完成
注册，后两者进入 ACTIVE。`scout2_camera` 和 `scout2_lidar` 也已完成注册，
不再出现旧 Conda 路径、缺少 `anyio` 或 Atlas 注册超时问题。

完整启动仍可能因 ROS 传感器没有实际数据而失败。最近一次验证中，Domain
42 可以看到 `/camera/camera/color/image_raw` 和 `/scan`，但在超时时间内
没有收到 RGB 或 LaserScan 消息。因此换用户后除了 Python 环境，还要确认
传感器驱动持续发布实际消息，而不只是 ROS 图中存在话题名称。

后续诊断确认传感器本身能够发布数据，但驱动由 `user` 用户启动、Robonix
由 `root` 用户启动时，Fast DDS 默认共享内存传输无法跨用户交付消息。
`ros2 topic list` 仍能显示话题名称，因此仅查看话题列表会产生“数据正常”
的误判。Scout2 部署现已在顶层 `env` 中设置：

```yaml
FASTDDS_BUILTIN_TRANSPORTS: UDPv4
```

该配置强制 Scout2 进程使用 UDPv4。实测 root 进程启用后，6 秒内收到 21
帧 RGB 和 60 帧 LaserScan。长期运行仍推荐让传感器驱动和 `rbnx boot` 使用
同一个 Linux 用户，并避免重复启动 Velodyne 驱动；当前主机曾同时存在两套
`velodyne_container`，应在正式启动前只保留一套。

再次执行 `rbnx boot --no-update-check` 时，Soma stage 1 的五个 primitive 均
进入 ACTIVE，证明该配置修复了本次相机和雷达收不到数据的问题。后续 Scene
Docker 镜像因安装了不兼容的 MCP 2.x 而未能注册；已将 Scene 依赖约束为
`mcp<2`，本机 `robonix-scene:latest` 镜像也已更新并通过 FastMCP 导入检查。
完整服务栈的再次启动尚未验证。

## 活动地图、保存地图与对象删除

`GET /api/maps/{map_id}/objects` 能返回对象，只表示 Scene 中存在该
`map_id` 的活动或快照对象分区，不代表 Mapping 磁盘上一定存在同名保存
地图。Mapping 的正式保存地图位于 `maps/<map_id>/`；运行中的 RTAB-Map
数据库位于 `maps/.runtime/<map_id>/`，不能当作已保存地图直接删除。

例如当前磁盘上只有保存地图 `maps/11/`，而 `scout2_main` 仅存在于
`maps/.runtime/scout2_main/`。此时调用 Scene 的
`POST /api/maps/delete` 会先调用 Mapping 删除地图，Mapping 返回
`no saved map 'scout2_main'`，Scene 因而不会继续删除对象。这与
`GET /api/maps/scout2_main/objects` 仍能返回活动对象并不矛盾。

8091 的删除路径是 `/api/delete`，不是 `/api/50107/delete`。正确请求为：

```bash
curl -i -X POST http://localhost:8091/api/delete \
  -H 'Content-Type: application/json' \
  -d '{"map_id":"11"}'
```

如果需要删除当前 `scout2_main` 地图及其对象，不能直接删除活动地图。安全
流程是：先通过 Scene 门面保存当前地图和对象快照，切换到另一张保存地图，
再调用统一删除接口：

```bash
# 1. 将当前运行地图及其 Scene 对象保存为 scout2_main。
curl -i -X POST http://localhost:50107/api/maps/save \
  -H 'Content-Type: application/json' \
  -d '{"map_id":"scout2_main","note":"checkpoint before delete"}'

# 2. 切换到另一张已经保存的地图；当前示例中可用的是 11。
curl -i -X POST http://localhost:50107/api/maps/load \
  -H 'Content-Type: application/json' \
  -d '{"map_id":"11","mode":"localization"}'

# 3. 删除 scout2_main 的保存地图、对象快照、元数据和标注。
curl -i -X POST http://localhost:50107/api/maps/delete \
  -H 'Content-Type: application/json' \
  -d '{"map_id":"scout2_main"}'
```

每一步都应先检查响应中的 `ok` 是否为 `true`，失败时不要继续下一步。命令
末尾不要重复粘贴 JSON；例如 `-d '{"map_id":"scout2_main"}'2_main"}'`
会造成无效或难以判断的请求内容。

## 停机后手动清理全部地图与 Scene 数据

该流程用于 Robonix、Mapping 和 Scene 已全部停止时，永久删除 Scout2 的
所有保存地图、运行地图、Scene 物体、地图标注、地图关联元数据和场景图
缓存。操作不可撤销；不会删除代码、Python 虚拟环境、生成文件或
`rbnx-boot/cache` 中的 provider 源码缓存。

### 1. 确认相关进程已经停止

```bash
docker ps -a --format '{{.Names}} {{.Status}}' \
  | grep -E 'robonix|mapping|scene' || true

ps -ef \
  | grep -E 'robonix-(atlas|soma|executor|pilot|liaison|vitals)|scene_service|mapping_rbnx|rtabmap' \
  | grep -v grep || true
```

如果仍有 `robonix_scene`、Mapping 容器、RTAB-Map 或 Robonix 系统进程，
先正常关闭；不要在数据库仍被进程打开时删除目录。

### 2. 设置并核对默认数据目录

```bash
MAPS=/data/robonix/examples/scout2/services/service-map-rbnx/maps
SCENE=/data/robonix/system/scene/rbnx-build/data/robonix

ls -la "$MAPS" "$SCENE"
find "$MAPS" -maxdepth 3 -print
find "$SCENE" -maxdepth 3 -print
```

默认情况下：

- `MAPS` 下包含 `maps/<map_id>/` 保存地图以及 `maps/.runtime/` 运行数据库。
- `$SCENE/scene_memory/` 包含 Milvus Lite 对象库；`objects.db` 实际是目录，
  不能只按普通数据库文件处理。
- `$SCENE/scene_maps/` 保存地图到对象快照分区的关联元数据。
- `$SCENE/scene_annotations/` 保存房间等地图标注。
- `$SCENE/scene_graph/cache/` 保存对象标题和关系等派生缓存。

### 3. 永久删除地图和 Scene 数据

```bash
# 删除所有保存地图及 .runtime 运行地图，但保留空 maps 目录本身。
find "$MAPS" -mindepth 1 -maxdepth 1 -exec rm -rf -- {} +

# 删除所有持久化物体、地图关联元数据、标注和场景图缓存。
rm -rf -- \
  "$SCENE/scene_memory" \
  "$SCENE/scene_maps" \
  "$SCENE/scene_annotations" \
  "$SCENE/scene_graph/cache"
```

### 4. 验证清理结果

```bash
find "$MAPS" -mindepth 1 -print

for path in \
  "$SCENE/scene_memory" \
  "$SCENE/scene_maps" \
  "$SCENE/scene_annotations" \
  "$SCENE/scene_graph/cache"
do
  [[ ! -e "$path" ]] || echo "still exists: $path"
done
```

第一条 `find` 应无输出，循环也不应打印 `still exists`。下次启动时 Mapping
和 Scene 会按需重新创建空目录、运行数据库和对象库，这是正常行为。

如果启动前设置过 `MAPPING_MAPS_DIR`、`MAPPING_RUNTIME_DB_DIR`、
`SCENE_DATA_DIR`、`SCENE_OBJECT_MEMORY_DB`、`SCENE_ANNOTATIONS_DIR`、
`SCENE_MAP_META_DIR` 或 `SCENE_GRAPH_CACHE_DIR`，实际数据可能位于覆盖路径。
必须先核对这些环境变量并清理其指向目录，不能仅清理上述默认位置。
