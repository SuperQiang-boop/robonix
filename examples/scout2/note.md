# 地图与物体信息持久化

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
