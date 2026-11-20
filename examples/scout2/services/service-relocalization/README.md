# service-relocalization

Scout2 平面重定位服务：全图/全航向 BBS 搜索、平面 ICP、独立扫描验证，
再通过 Atlas 调用 mapping 的 `pose_estimate` 并用新扫描复核。
服务不主动移动小车；只有交接和持续验证通过才发布 `localization_valid=true`。

## 单配置启动

建图与定位都使用部署根目录的 `robonix_manifest.yaml`，命令见
[scout2-start.md](../../scout2-start.md)。不需要生成第二份定位清单。

- `mapping.config.map_mode: auto`：没有保存地图时建图，有保存地图时选择最新地图定位。
- 自动选图按地图目录修改时间排序，同时间按 map ID 排序；数据库在加载时另行校验。
- mapping 的 `map_id` 在 auto 模式下只命名首次空库建图；不会覆盖最新用户地图的选择。
- 重定位配置不固定 map ID；读取 mapping 实际加载的 ID、generation、数据库 SHA-256。
- `relocalization.config.auto_relocalize` 默认 `true`，改为 `false` 后只接受手动请求。

mapping 模式下重定位服务等待，不生成索引、不创建自动请求；手动请求也需先加载定位地图。
mapping 继续实时建图，按 30 秒周期自动保存，正常退出再保存。保存不是手动按钮操作；
不完整或不连通的导出不会覆盖上一份有效地图。仅保存地图不会在运行中自动切换定位模式。
有保存地图后，下次启动仍使用同一份清单；也可通过现有 `load_map` 加载指定地图。
开启协调时，`switch_mode(localization)` 会先保存当前命名地图，再经过同一检查加载路径。

自动请求在 localization 地图就绪稳定后发起，每次加载最多尝试 3 次。
关闭开关不会关闭查询接口。`active=true` 只说明服务活动，`accepted=true` 只说明受理；
定位无效时 navigation 拒绝自主任务和速度输出。稳定建图期间导航守卫不持续发送零速度
干扰遥控，但仍拒绝 Nav2 输出。地图切换或定位失效仍要求停车。

## 构建与运行环境

在 Scout2 部署目录执行 `rbnx build -f robonix_manifest.yaml --no-update-check`。
构建需 ROS 2 Humble、Python 3.10、rbnx、uv、colcon、C++17、PCL、Eigen3 和 pybind11。
主机运行原生重定位扩展，mapping/Nav2 可在 Docker 中运行。

`config/relocalization.json` 只保留雷达时序和标定，不放自动触发开关或固定地图索引。
`config/relocalization.example.json` 中 null 字段必须填写；不能直接用于实车。

| 字段 | 要求 |
| --- | --- |
| `capture.scan_duration_ns` | 驱动确认的扫描跨度，不能从帧率猜测 |
| `capture.header_is_end` | 驱动确认 header 是否表示扫描结束 |
| `capture.voxel_size_m` | 当前 Scout2 配置为 0.075 米 |
| `base_from_sensor` | 4×4 行优先外参，与 URDF、实时 TF 和保存扫描一致 |
| `registration.base_z/roll/pitch` | 经确认的固定高度和倾角 |
| `search/registration/matching` | 可选算法参数 |

没有匹配索引时，从当前加载身份对应的稳定保存快照自动准备
`data/<map_id>/<key>/`。导出器拒绝运行中的数据库、SQLite sidecar、断图、
缺少优化位姿/扫描的数据；索引失败只报告错误并维持定位无效，不中断 mapping。
索引准备失败会按 1、2、4 秒递增重试，间隔上限 30 秒；依赖恢复后可继续自动或手动定位，
不消耗每次加载最多 3 次的自动定位请求次数。
常规 `rbnx build` 同时构建原生匹配后端和快照导出器。导出器默认使用本机
`robonix-mapping:latest`，可用 `ROBONIX_MAPPING_IMAGE` 或优先级更高的
`RELOCALIZATION_BUILD_IMAGE` 指定；不自动拉取镜像。构建记录实际镜像 ID，
镜像变化时重新编译并通过导出测试后才更新记录，避免继续引用已删除的旧镜像。
如清理或替换镜像后启动时报导出器镜像不可用，在 Scout2 目录重新执行上述构建即可；
无需删除地图、物体或手工修改 `image-id`。单独原生构建仍可使用
`scripts/build_search.sh` 与未设置 `RELOCALIZATION_BUILD_IMAGE` 的 `scripts/build_backend.sh`。

mapping、navigation、relocalization 在同一 Linux 主机共享协调目录。
清单 env 的 `RELOCALIZATION_COORDINATION_DIR`、`RELOCALIZATION_PACKAGE_ROOT`
与各服务 config 路径应一致；相对路径均以 `RBNX_INVOCATION_CWD` 为基准。
Docker 将协调目录挂载为可写、包目录只读。原子记录、flock 和心跳绑定每次实际加载，
同图重载也会撤销旧定位。没有配置协调的其他部署不会隐式启用这套检查。

## 接口和测试

手动请求只需 `relocalize_by_map(map_id)`，保存 `request_id` 后轮询空请求 `get_status`；
`busy=false` 且 `localization_valid=true` 才表示本次请求成功。
完整接口、结果字段及示例见 [relocalization.md](relocalization.md) 和
[CAPABILITY.md](CAPABILITY.md)。接口通过 Atlas 发现，禁止硬编码动态 gRPC 端口。

```bash
bash services/service-relocalization/scripts/smoke.sh
bash services/service-relocalization/scripts/smoke.sh --production-e2e
```

隔离联调用合成地图/ROS 输入、真实 Atlas、Driver、原生匹配和 RPC，
覆盖空图库等待、自动定位、同图重载、关闭自动后手动请求及资源清理，不连接实车。
实车搬放定位、误定位拒绝和受控导航仍需实车验收。

环境变量：`ROBONIX_SOURCE_PATH` 指定源码；`RELOCALIZATION_PYTHON` 选择构建解释器；
`RELOCALIZATION_ROS_SETUP` 选择 ROS setup；`RBNX_INVOCATION_CWD` 指定部署目录。
运行继承 `ROBONIX_ATLAS`、`ROS_DOMAIN_ID`、`RMW_IMPLEMENTATION` 和 Fast DDS 配置。
只读诊断可用 `relocalization_rbnx.calibration`、`probe`、`map_check` 的 `--help`。
