# Scout2 地图服务与导航服务分析

> 第 1–17 节记录改造前的分析与方案；当前实现和验证结果见第 18 节。特别注意：周期性保存不等于逐帧零丢失，加载数据库也不等于完成机器人重定位。

## 1. 总体数据流

```text
Scout2 LiDAR + 底盘里程计
        │
        ▼
service-map-rbnx
  └─ RTAB-Map SLAM
       ├─ 实时二维栅格地图：/map
       ├─ 三维点云：/rtabmap/cloud_map
       ├─ SLAM 位姿：map -> odom
       └─ 地图数据库：SQLite rtabmap.db
        │
        ▼
service-navigation-rbnx
  └─ Nav2
       ├─ StaticLayer 订阅地图
       ├─ ObstacleLayer 订阅实时激光
       ├─ 生成全局/局部代价地图
       └─ 执行路径规划和导航
```

## 2. 当前启用的地图服务

在 `robonix_manifest.yaml` 中，地图服务配置为：

- 服务目录：`./services/service-map-rbnx`
- SLAM 算法：RTAB-Map
- 输入：
  - `lidar3d: scout2_lidar`
  - `odom: scout2_chassis`
- `deskew_lidar: true`
- 当前 `rgb`、`depth` 配置被注释，因此实际主要是 3D LiDAR 加底盘 odom 建图。

RTAB-Map 对外发布：

- `robonix/service/map/occupancy_grid`
- `robonix/service/map/pointcloud`
- `robonix/service/map/pose`
- `robonix/service/map/odom`

RTAB-Map 的地图数据库使用 SQLite 格式。

## 3. 导航服务如何使用地图

导航服务配置在 `robonix_manifest.yaml` 中：

```yaml
provider_ids:
  map: mapping
  odom: scout2_chassis
  scan: scout2_lidar
  scan_cloud: scout2_lidar
```

Nav2 通过 Atlas 找到地图服务发布的 `robonix/service/map/occupancy_grid`，并将其映射为 Nav2 使用的 `/map` 话题。

`config/nav2_params.yaml` 中的全局代价地图配置为：

```yaml
static_layer:
  plugin: "nav2_costmap_2d::StaticLayer"
  map_topic: "__ROBONIX_MAP_TOPIC__"
  map_subscribe_transient_local: true
```

因此：

- 全局代价地图使用 RTAB-Map 发布的实时地图；
- 全局和局部代价地图还会叠加当前激光障碍物；
- 导航服务不负责建图和地图持久化；
- 导航服务只消费地图、odom 和激光数据进行规划与控制。

## 4. 建图过程中地图保存在哪里

当前 `robonix_manifest.yaml` 没有配置 `map_id` 或 `map_mode`，所以使用默认配置：

```yaml
map_mode: mapping
```

每次启动都会创建一个新的运行时 RTAB-Map 数据库。

### Docker 模式

运行时数据库默认位于容器内：

```text
/tmp/robonix-mapping-runtime/
```

容器退出后该路径通常随容器销毁，不能作为永久地图目录。

### Native 模式

运行时数据库默认位于：

```text
services/service-map-rbnx/rbnx-build/runtime/
```

Native 启动脚本会在启动时清理该目录中的旧 `.db`、`.db-wal` 和 `.db-shm` 文件。因此它是当前建图会话的临时数据库目录，不是最终地图仓库。

## 5. 什么时候保存成可复用地图

建图过程中不会自动按地图名称保存永久地图。需要调用：

```text
robonix/service/map/save_map
```

也可以通过地图 Web UI 的 Save 操作执行。

例如保存为 `lab_3f` 后，地图服务会在以下目录生成文件：

```text
{MAPPING_MAPS_DIR}/lab_3f/
```

包含：

```text
rtabmap.db       RTAB-Map 的 SLAM 图和定位数据
occupancy.pgm   2D 栅格图
occupancy.yaml  栅格图元数据
occupancy.png   栅格图预览
cloud.pcd       融合后的三维点云
meta.yaml       地图 ID、分辨率、原点等信息
```

默认目录：

- Native 模式：`services/service-map-rbnx/maps/<map_id>/`
- Docker 模式容器内：`/mapping/maps/<map_id>/`

Docker 启动时会把 `services/service-map-rbnx` 挂载到容器 `/mapping`，所以默认对应的宿主机目录仍然是：

```text
services/service-map-rbnx/maps/<map_id>/
```

也可以用环境变量指定其他持久化目录：

```bash
export MAPPING_MAPS_DIR=/data/robot_maps
```

## 6. `save_map` 的保存流程

`save_map(map_id)` 大致执行以下步骤：

1. 找到当前 RTAB-Map 的运行时数据库；
2. 请求 RTAB-Map 刷新数据库；
3. 使用 SQLite backup API 做一致性快照；
4. 写入临时 staging 目录；
5. 发布完整 `/map` 和点云预览；
6. 生成 `occupancy.*`、`cloud.pcd`、`meta.yaml`；
7. 完整性检查通过后，原子发布到 `MAPPING_MAPS_DIR/<map_id>/`。

已保存的地图被视为空间地图 artifact，重复使用相同 `map_id` 不会直接覆盖已有文件。

## 7. 重新使用已保存地图

启动配置可以写成：

```yaml
map_id: lab_3f
map_mode: localization
```

此时：

1. 地图服务读取 `MAPPING_MAPS_DIR/lab_3f/rtabmap.db`；
2. 将保存的数据库复制到运行时数据库目录；
3. RTAB-Map 使用运行时副本进行定位；
4. 保存的 `rtabmap.db` 保持不变；
5. 地图坐标系在重启后保持稳定；
6. Nav2 继续从地图服务收到 `/map`。

## 8. 路径总结

| 类型 | 位置 | 作用 |
|---|---|---|
| 实时建图数据库 | Docker：`/tmp/robonix-mapping-runtime/`；Native：`services/service-map-rbnx/rbnx-build/runtime/` | 当前会话使用，重启通常清理 |
| 已保存地图 | `maps/<map_id>/rtabmap.db` | 可复用、可定位 |
| 栅格地图文件 | `maps/<map_id>/occupancy.pgm/.yaml` | 离线查看或其他 map server 使用 |
| 实时地图话题 | `/map` 或 Robonix map capability | Nav2 在线导航使用 |

## 9. 最终结论

当前配置启动后，机器人是在运行时数据库和 ROS 话题中持续建图。只有显式调用 `save_map(map_id)`，才会把地图持久化到：

```text
services/service-map-rbnx/maps/<map_id>/
```

其中 `rtabmap.db` 是主要的空间地图数据源，`occupancy.pgm/.yaml`、`occupancy.png`、`cloud.pcd` 和 `meta.yaml` 是配套的可查看、可交换文件。Nav2 在线运行时直接订阅实时地图话题，并不是直接读取这些文件。

---

# 持久化建图与重启恢复改造方案

## 10. 当前实现的限制

当前实现还不能直接满足“Docker 重启后恢复地图，并继续增量建图”的需求：

1. `map_mode: mapping` 每次启动都会创建新的运行时数据库，并设置 `reset_map=true`，不会继续使用上次地图。
2. `load_map()` 即使传入 `mode=mapping`，内部也会强制改成 `localization`。
3. `save_map()` 将已保存地图视为不可变文件，同一个 `map_id` 已存在时会拒绝再次保存。
4. Docker 模式的运行时数据库默认在容器 `/tmp/robonix-mapping-runtime/`，容器删除后会丢失。
5. `stop.sh` 使用 `docker rm -f`，而容器清理时先停止 RTAB-Map、再停止 bridge，无法保证退出前完成地图快照。

相关实现位置：

- `services/service-map-rbnx/src/mapping_rbnx/atlas_bridge.py`
- `services/service-map-rbnx/src/mapping_rbnx/map_ops.py`
- `services/service-map-rbnx/launch/rtabmap_2d.launch.py`
- `services/service-map-rbnx/scripts/start.sh`
- `services/service-map-rbnx/scripts/stop.sh`

## 11. 推荐的数据目录

使用固定的 `map_id`，并把实时写入数据库和对外发布的地图快照分开：

```text
/robot-data/
├── maps/
│   └── scout2_main/
│       ├── rtabmap.db
│       ├── occupancy.pgm
│       ├── occupancy.yaml
│       ├── occupancy.png
│       ├── cloud.pcd
│       └── meta.yaml
└── runtime/
    └── scout2_main/
        ├── rtabmap.db
        └── rtabmap.db.back
```

- `runtime/scout2_main/rtabmap.db` 是 RTAB-Map 当前持续写入的活动数据库。
- `maps/scout2_main/rtabmap.db` 是最近一次成功 checkpoint 的稳定地图。
- 两者都必须位于 Docker bind mount 对应的宿主机目录中。
- `occupancy.*` 和 `cloud.pcd` 不需要每帧写入，可按时间间隔或关闭时生成。

## 12. 部署配置

在 `robonix_manifest.yaml` 中使用固定地图 ID：

```yaml
- name: mapping
  path: ./services/service-map-rbnx
  config:
    map_id: scout2_main
    map_mode: mapping
    reset_map: false
    webui_port: 8091
    params_file: config/rtabmap_params.yaml
    sensor_providers:
      lidar3d: scout2_lidar
      odom: scout2_chassis
```

Docker 模式建议显式指定宿主机数据目录：

```bash
export ROBONIX_MAPPING_FORCE=docker
export MAPPING_MAPS_DIR=/data/robonix/scout2/maps
export MAPPING_RUNTIME_DB_DIR=/data/robonix/scout2/runtime
```

当前 `scripts/start.sh` 只显式处理 `MAPPING_MAPS_DIR`，需要增加 `MAPPING_RUNTIME_DB_DIR` 的环境变量传递和 bind mount。若使用 `rbnx boot`，主要应修改 `scripts/start.sh`，不能只修改 `docker/compose.yaml`，因为默认启动路径是 `docker run`。

## 13. 地图服务需要修改的逻辑

### 13.1 `_resolve_persistence()`

当 `map_mode=mapping` 且配置了 `map_id` 时：

- 如果存在 runtime 数据库，直接继续使用；
- 如果 runtime 不存在但 `maps/<map_id>/rtabmap.db` 存在，将其复制到 runtime；
- 如果两者都不存在，创建新的 runtime 数据库；
- 不再默认设置 `reset_map=true`。

`map_mode=localization` 可以继续保留为只读定位模式。

### 13.2 RTAB-Map 启动参数

`rtabmap_2d.launch.py` 只在明确请求重新建图时传入 `--delete_db_on_start`。已有数据库以 mapping 模式启动时不能删除数据库，并应保持：

```text
Mem/IncrementalMemory=true
```

这样 RTAB-Map 可以在已有地图图结构上继续增加新节点。

### 13.3 `load_map()`

如果调用方请求 `mode=mapping`，不能再强制转换为 localization。另一种可行流程是先以 localization 加载，再调用已有的 `switch_mode("mapping")`，但这要求后续保存逻辑支持覆盖同一个 `map_id`。

### 13.4 `save_map()`

需要删除“已有 `map_id` 就拒绝保存”的限制：

1. 调用 RTAB-Map 的 `/backup`；
2. 将数据库备份到 staging 目录；
3. 重新生成栅格图、点云和 metadata；
4. 对已有 `maps/<map_id>/` 做原子更新；
5. 不能替换或删除正在被 RTAB-Map 使用的 runtime 数据库；
6. 保留 `generation` 文件，避免地图生命周期标识丢失。

### 13.5 周期性 checkpoint 和退出保存

RTAB-Map 的 runtime 数据库并不保证每帧立即落盘，部分节点驻留在工作内存中；因此需要周期性调用 `/backup` 刷新内存，再由 `save_map("scout2_main")` 更新稳定快照。保存操作需要互斥并与引擎协调。项目正常关闭时还应执行一次 checkpoint；强制退出或断电可能丢失最近一次成功刷新之后的数据。

### 13.6 Docker 停止顺序

正常停止时应按以下顺序处理：

1. 通知 bridge 执行地图保存；
2. 等待地图快照完成；
3. 再停止 RTAB-Map；
4. 使用足够长的 `docker stop` 超时时间；
5. 不要把 `docker rm -f` 作为正常关闭路径。

## 14. 运行流程

首次启动：

```text
没有 runtime 数据库
    ↓
创建新的 runtime 数据库
    ↓
RTAB-Map 持续建图
    ↓
周期性保存到 maps/scout2_main/
```

项目关闭：

```text
停止请求
    ↓
调用 RTAB-Map backup
    ↓
更新 maps/scout2_main/ 下的稳定地图文件
    ↓
停止 RTAB-Map 和 Docker 容器
```

项目重启：

```text
Docker 重新挂载宿主机数据目录
    ↓
发现 runtime/scout2_main/rtabmap.db
    ↓
以 mapping 模式打开已有数据库
    ↓
继续在原地图坐标系上建图
```

进入新场景后，RTAB-Map 继续增加新节点、融合新场景和回环，并周期性 checkpoint 到同一个 `map_id`。

如果需要从只定位切换到继续建图，可以使用：

```text
load_map("scout2_main", localization)
    ↓
switch_mode("mapping")
    ↓
运动到新场景
    ↓
save_map("scout2_main")
```

## 15. Nav2 侧注意事项

Nav2 已经通过地图 capability 订阅实时地图话题，而不是直接读取 `occupancy.yaml`。只要地图服务重启后继续发布同一个 `map_id` 对应的 `/map`，Nav2 就可以继续使用。

需要额外保证：

- 地图加载完成并发布非空 `/map` 后，Nav2 才允许接收导航目标；
- 加载地图、切换模式、重置地图期间暂停导航任务；
- 不要使用 `reset_map`，否则地图原点会改变；
- 持久化的导航目标应关联 `map_id` 和现有的地图生命周期 generation；
- 如果 RTAB-Map 发生大范围回环优化，原有坐标可能发生变化，需要重新确认目标点。

## 16. 验证顺序

1. 空数据目录启动，确认 runtime 数据库出现在宿主机。
2. 建图后调用 `save_map("scout2_main")`，确认生成完整地图文件。
3. 正常停止 Docker，确认关闭前完成 checkpoint。
4. 重启 Docker，确认 `/map` 非空且地图坐标连续。
5. 进入新区域继续建图，再次保存同一个 `map_id`。
6. 验证数据库、栅格图和点云均被更新且没有损坏。
7. 启动 Nav2，确认全局代价地图加载并可导航到已保存位置。
8. 额外测试 Docker 强制重启，确认 runtime 数据库可以恢复。

## 17. 推荐结论

推荐采用“持久化 runtime 数据库 + 可覆盖的稳定快照 + 关闭时 checkpoint”的方案。它比直接让 RTAB-Map 写入最终 `maps/<map_id>/rtabmap.db` 更安全，可以避免保存时原子替换正在使用的数据库。

## 18. 已实现的 Docker 持久化建图

Scout2 的 `robonix_manifest.yaml` 已强制地图服务使用 Docker，并设置：

```yaml
map_id: scout2_main
map_mode: mapping
reset_map: false
checkpoint_interval_s: 60
```

继续在 Scout2 目录运行 `rbnx build`、`rbnx boot`，无需手动保存后才能再次建图。

### 保存位置与恢复行为

- 宿主机运行数据库：`services/service-map-rbnx/maps/.runtime/scout2_main/rtabmap.db`。
- 宿主机稳定快照：`services/service-map-rbnx/maps/scout2_main/rtabmap.db`。
- 容器对应路径：`/mapping/maps/.runtime/scout2_main/rtabmap.db` 和 `/mapping/maps/scout2_main/rtabmap.db`。目录由宿主机挂载，删除容器或重建镜像不会删除宿主机地图。
- 首次启动建立数据库，后续启动优先复用同名运行数据库；缺失时由稳定快照恢复。损坏的运行数据库及其 SQLite 附属文件保留到 `previous-*` 子目录后再恢复；没有可用快照则启动报错，不静默创建空图。
- `save_map` 允许同名更新，原子替换稳定数据库文件，保留 generation、其他地图文件以及上一版 `rtabmap.previous.db`；不替换引擎正在使用的数据库。
- 预览图和点云尽力导出，失败会报告，但不阻止数据库保存；预览可能滞后，恢复地图以数据库为准。

### 自动保存与退出

上一次保存结束后等待 30 秒再保存：调用 RTAB-Map `/backup` 刷新工作内存，并优先原子移动已生成的 `.back` 文件发布快照，跨文件系统时才回退到 SQLite backup；发布旧版本也使用原子改名，避免重复复制多 GB 数据库。保存、加载、模式切换共用互斥锁。正常退出先停止周期任务、执行最后一次保存，再通知引擎关闭数据库；Docker 改用优雅停止，不再直接强制删除运行容器。

`checkpoint_interval_s: 0` 仅关闭周期保存，不关闭退出保存。`ROBONIX_MAPPING_STOP_TIMEOUT_S` 默认 180 秒，大地图应留足退出时间。周期保存并非严格实时逐帧落盘：异常中断可能丢失最后一次成功刷新以后的增量，保存失败时丢失窗口还会扩大。部署已设置 `DbSqlite3/InMemory=false`、`JournalMode=0`、`Synchronous=2`，提高 SQLite 崩溃恢复可靠性，但不承诺断电零丢失。

不要把 `reset_map` 长期设为 `true`，否则每次启动都会新建地图；不要让多个服务同时写同一个运行数据库。构建和清理时保留宿主机 `maps` 数据。Web UI 的删除操作只删除发布快照，不清除持久化运行数据库；需要重新建图时应明确使用 reset。

### 定位和导航边界

重启继续使用 `mapping`（增量建图），并通过 `Mem/InitWMWithAllNodes=true` 载入历史节点供匹配，代价是更多启动时间和内存。建议从旧地图中有重叠观测的位置启动，先确认位姿、激光和旧地图对齐，再进入新区域；若搬动过机器人，可先定位并设置初始位姿，再切回建图。

现有 `load_map` 仍先加载定位副本；随后 `switch_mode(mapping)` 会把该副本转入固定持久化路径再继续建图。纯定位退出不会自动用旧副本覆盖建图快照。恢复数据库本身不保证机器人已经重定位，回环优化也可能改变旧目标坐标。本次未新增 Nav2 的定位确认门控，实机导航必须另外验证。

### 验证结果

- `rbnx build -p services/service-map-rbnx --no-update-check` 已完成，Docker 镜像和 ROS map 接口构建成功。
- 67 项离线单元测试通过，覆盖同名恢复、损坏数据保留与恢复、重复保存、失败不覆盖、周期任务跟随当前地图、定位转建图和退出保存幂等。
- 隔离 Docker（`--network none`）内通过实际 `start_engine.sh` 使用真实 RTAB-Map 与模拟激光/里程计，最终复测图节点从 **50 → 100 → 151**；正常重启和引擎强制中断后均保留旧节点并继续新增，恢复后在新扫描前即可重新发布 `/map`，生成了地图预览。退出通过独立进程组向 ROS 启动程序及子进程传递信号。脚本为 `services/service-map-rbnx/scripts/test_persistence_ros.py`。
- 尚未运行连接底盘的完整 `rbnx boot` 和实机导航；上述测试不发送任何底盘运动命令，也不等同于真实断电测试。

## 19. `rbnx boot` 实机验证路径

在未额外设置持久化目录环境变量时，Scout2 通过 `rbnx boot` 启动 Docker 地图服务后，宿主机的实际路径是：

```text
/home/user/wfw/robonix/examples/scout2/services/service-map-rbnx/maps/.runtime/scout2_main/rtabmap.db
```

这是 RTAB-Map 当前持续写入的运行数据库，可能同时产生 `rtabmap.db-wal`、`rtabmap.db-shm` 和 `rtabmap.db.back`。稳定快照位于：

```text
/home/user/wfw/robonix/examples/scout2/services/service-map-rbnx/maps/scout2_main/rtabmap.db
```

同一目录还可能包含 `occupancy.pgm`、`occupancy.yaml`、`occupancy.png`、`cloud.pcd`、`meta.yaml`、`generation` 和 `rtabmap.previous.db`。容器内对应路径分别是 `/mapping/maps/.runtime/scout2_main/rtabmap.db` 和 `/mapping/maps/scout2_main/rtabmap.db`；`/mapping` 由宿主机的 `services/service-map-rbnx` 目录挂载，因此删除或重建容器不会删除这些文件。

重启时配置仍为 `map_mode: mapping`、`reset_map: false`：如果已经存在保存快照，服务先把 `maps/scout2_main/rtabmap.db` 事务性复制到 `.runtime/scout2_main/rtabmap.db`，再继续增量建图，因此启动后的 `/map` 从最近一次显式/周期保存的地图开始。首次尚未保存快照时，服务保留运行数据库用于继续建图；运行数据库损坏时先保留到 `previous-*` 子目录，再从保存快照恢复。首次实机启动前该目录可能尚不存在，建图运行后可检查：

```bash
find services/service-map-rbnx/maps -maxdepth 3 -type f -ls
```

如果启动环境显式设置了 `MAPPING_MAPS_DIR` 或 `MAPPING_RUNTIME_DB_DIR`，对应宿主机路径会改为这些变量指定的目录。

## 20. 当前代码验收结论

按当前 `robonix_manifest.yaml` 和地图服务实现，自动建图与持久化链路已经接通：

| 项目 | 当前结论 | 依据 |
|---|---|---|
| 自动建图 | 满足 | `map_mode: mapping`、`reset_map: false`，RTAB-Map 使用持久化 runtime 数据库并保持 `Mem/IncrementalMemory=true` |
| 自动保存 | 满足 | `checkpoint_interval_s: 30`；每次 checkpoint 先调用 RTAB-Map `/backup`，再原子发布快照；正常退出还会再保存一次 |
| 本地持久化 | 满足 | Docker 将 `services/service-map-rbnx` 挂载为 `/mapping`，runtime 数据库和稳定快照位于宿主机 `maps` 目录 |
| 电源重启后恢复 | 满足保存快照恢复 | 已有保存快照时启动先从 `maps/scout2_main/rtabmap.db` 重建 runtime，再继续 mapping；首次尚未保存时才保留 runtime |
| 最近一帧绝对不丢失 | 不保证 | 周期保存间隔为 60 秒；突然断电可能丢失上一次成功 checkpoint 之后的增量 |
| 恢复后自动重定位 | 不保证 | 数据库可以恢复并继续 mapping，但机器人必须从旧地图有重叠观测的位置启动并确认位姿对齐 |

因此，代码已经满足“自动建图、自动保存到本地、重启后加载历史地图并继续建图”的设计目标；实机仍需验证底盘传感器、Docker 挂载、RTAB-Map 位姿对齐和真实断电行为。正常执行 `rbnx shutdown` 可以触发退出保存，直接切断电源只能依赖最近一次成功 checkpoint。

实机日志若出现 `Odometry is reset (identity pose detected). Increment map id`，说明底盘重连期间的零位姿触发了 RTAB-Map 内部切图；数据库节点仍存在，但 `/map` 只发布新内部地图，看起来像历史地图丢失。Scout2 将 `Odom/ResetCountdown` 设为较大阈值 `1000000`，避免高频身份里程计在启动阶段耗尽计数并切换内部地图。

### 50107 显示实时地图还是历史地图

重启日志出现以下内容时，说明历史数据库已经恢复，而不是从空库开始：

```text
restored runtime database from checkpoint /mapping/maps/scout2_main/rtabmap.db
Using database .../scout2_main/rtabmap.db
2D occupancy grid map loaded (...)
Working Memory = ...
```

50107 默认订阅映射服务持续发布的 `/map`（通过 `robonix/service/map/occupancy_grid`）。Scout2 启动配置是 `map_mode: mapping`，并且 `Mem/IncrementalMemory=true`，因此 RTAB-Map 会在恢复历史数据库后继续接收激光数据、更新地图并持续发布；页面看到的是“恢复历史地图后继续增长的实时地图”，不是冻结的磁盘快照。只要日志没有再次出现 `Increment map id`，且数据库节点数持续增长，历史地图数据就仍在当前地图中。

如果需要在 50107 固定查看重启前的历史地图，应在页面地图列表中对 `scout2_main` 执行 **Load**，选择 `localization` 模式。该模式加载保存快照并关闭增量建图，页面显示固定历史地图；切回 `mapping` 后才会继续采集并更新地图。磁盘快照仍位于 `services/service-map-rbnx/maps/scout2_main/rtabmap.db`，50107 的实时显示不代表该文件被覆盖或丢失。

建议实机验证顺序：启动后确认 `maps/.runtime/scout2_main/rtabmap.db` 出现；运行超过 60 秒后确认 `maps/scout2_main/rtabmap.db` 的修改时间和大小变化；执行一次正常关闭并重新 `rbnx boot`，确认 `/map` 非空且节点数或地图范围延续；最后在充分 checkpoint 后进行一次电源重启，记录恢复结果以及是否丢失最近增量。
