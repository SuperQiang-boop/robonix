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

本次相关代码已执行 Python 语法检查、启动脚本 `bash -n` 检查及 `git diff --check`。当时运行环境缺少 Python `starlette` 依赖，无法完成 web 测试；Docker socket 不可用，也未完成容器端到端重启验证。
