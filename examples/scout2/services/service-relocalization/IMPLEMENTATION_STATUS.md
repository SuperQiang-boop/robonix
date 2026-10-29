# 重定位服务当前状态

更新：2026-10-29。部署目录：`/data/robonix/examples/scout2`。
当前设计与接口示例见 [relocalization.md](relocalization.md)；启动命令见
[scout2-start.md](../../scout2-start.md)。本记录只保留现行单配置流程。

## 已实现

- 建图与定位共用 `robonix_manifest.yaml`，`mapping.config.map_mode: auto`。
- 空图库启动 mapping：实时建图、30 秒周期自动保存和正常退出保存保持开启。
  重定位服务启动后等待，不准备索引、不自动发起请求，不阻塞建图。
- 已保存地图存在时，按地图目录修改时间选最新地图进入 localization，同时间按 ID 排序。
  `scout2_main` 只在空图库时命名首次建图；显式选择地图可用原清单的 localization/map_id。
- 重定位不固定地图 ID 或索引：等待 mapping 实际完成加载和占据图检查，再读取
  map_id、generation、数据库 SHA-256，选择或生成完全匹配的索引。
- `relocalization.config.auto_relocalize` 默认 true；每次加载最多自动尝试 3 次。
  false 时只监控地图、发布状态，等待现有 `relocalize_by_map(map_id)` RPC。
  `get_status` 查询 request_id、busy、localization_valid、位姿、residual_m、overlap、detail。
- 索引准备失败按 1、2、4 秒退避重试，间隔上限 30 秒；依赖恢复后自动或手动请求可继续，
  不会在准备失败 3 次后永久停住，也不占用自动定位请求次数。
- 标定 profile 不配置自动开关；当前部署 profile 没有固定 index/map_id。
  所有配置相对路径以部署目录解析；Docker 显式共享可写协调目录和只读包目录。
- 常规构建同时重建快照导出器，绑定实际 mapping 镜像 ID；镜像变化强制重新编译，
  测试成功才发布 image-id，失败撤销记录。已修复旧镜像被删除导致索引准备失败、
  自动请求尚未创建的问题；缺失镜像的诊断包含统一构建命令。
- Scene 从 mapping lifecycle 获取实际地图身份，不静态绑定 scout2_main。
- 无效定位阻止自主导航；稳定建图时守卫不持续发送零速度干扰遥控。
  地图切换和失效仍拒绝 Nav2 输出、停车并撤销旧结果。
- mapping/reset 会发布建图状态。开启协调时切换 localization 先保存再经过已验证的加载路径，
  不把仍在修改的运行库伪装为有效定位快照。

## 本轮验证

- mapping Python 回归：76/76 通过，含空图库、最新自定义地图、自动保存和模式切换。
- navigation Python 回归：47/47 通过，含 mapping 遥控保护、过期/切换停车、
  两个 Docker 启动脚本相对共享路径挂载，以及无协调配置时不隐式启用。
- relocalization 单元测试：85/85 通过，含单清单自动/手动配置、索引准备恢复、
  失败结束后退避计时、30 秒间隔上限及自动请求次数独立回归。
- Scene 地图、标注与自动快照回归：74/74 通过，保留真实线程池调用。
- 本轮原生匹配 CTest：4/4；容器导出器 CTest：5/5，含真实 RTAB-Map 导出测试。
- 当前 scout2_main 实际生成并加载索引：generation=1，98 个优化节点，1 个连通分量，
  无排除节点，304776 个索引点。数据库摘要与保存快照一致，全部扫描外参与 profile 一致。
  节点 1、93、182 的保存扫描离线几何检查通过；这不能替代搬放后的实时定位验收。
- 隔离生产入口 E2E：1/1 通过，覆盖无地图等待、自动定位、同图重载重新定位、
  auto_relocalize=false 后手动 map_id 请求及 request_id 关联、停用资源回收。
  使用隔离 Atlas、合成地图/ROS 输入和原生匹配，不连接实车。
- 在本部署目录完整执行 `rbnx build -f robonix_manifest.yaml --no-update-check`：15/15 成功。
  首次沙箱运行受 Docker socket、uv 缓存、Scene 目录权限限制；授权重跑后全部通过。
- Python 语法、YAML、共享路径一致性、导航速度上限和 `git diff --check` 通过。

日志：`/tmp/scout2-single-manifest-build.log`、`/tmp/scout2-relocalization-e2e.log`、
`/tmp/scout2-mapping-unit.log`、`/tmp/scout2-navigation-unit.log`；
提交前复核日志：`/tmp/scout2-precommit-relocalization.log`（85 项单元测试）、
`/tmp/scout2-precommit-final-relocalization.log`（隔离生产入口 E2E）。

## 下次执行

不用清理历史地图，不用生成另一份 localization manifest。本轮已构建完成；
按 `scout2-start.md` 中相同清单启动后，检查实际 mapping 模式和地图 ID。
在 localization 模式保持小车静止，等待自动请求结束，核对 request_id、busy=false、
localization_valid=true、位姿/残差/重叠率及持续观测新鲜度。
自动开关关闭时手动调用相同接口，示例见 relocalization.md。
配置开关修改只需重启；源码变化后才需要重新构建。

实车搬放召回率、误定位拒绝、耗时统计及受控短距离导航仍待验收。
本轮没有启动实车、发送运动命令、删除地图或提交代码。
