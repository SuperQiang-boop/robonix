# C++ 地图导出

`relocalization_export` 使用 RTAB-Map 0.23.7 的 DBDriver、SensorData 和
LaserScan API 只读导出稳定数据库，保留优化位姿、扫描外参、完整图连通性
和同一快照的保存栅格。不能回退到 odom 位姿，断图需要明确锚点才可作诊断。

由 `../scripts/build_backend.sh` 构建并运行 CTest；容器构建用
`RELOCALIZATION_BUILD_IMAGE` 显式选择本地镜像，自动记录不可变镜像 ID。
原生构建需在 CMAKE_PREFIX_PATH 中提供匹配的 RTAB-Map 开发库。
离线准备使用 `../scripts/prepare.sh`，详见主 README。

`cloud.bin` 和 `scans.bin` 为小端 float32 XYZ，分别位于地图和底盘坐标；
`export.json` 的逐节点 offset/count 以点为单位，矩阵为 3×4 行优先。
`grid.bin` 为 int8 行优先占据栅格（-1/0/100），原点和分辨率来自数据库。
输出只是中间文件；只有 Python 校验并原子发布的 `index.json` 可表示索引就绪。

`relocalization_bbs` 包含 BBS 多候选搜索及 PCL 最近邻支持的平面约束
ICP。精配准固定调用方给定的高度、roll、pitch，只优化 x/y/yaw；通过
重叠率、RMSE、空间分布、收敛和候选边界门限拒绝不可靠匹配。固定姿态
评估接口可评分另一帧而不再次拟合。两者都不能单独认定机器人定位有效。

计算代码通过 pybind11 接入 Python，释放 GIL，并支持取消和单调时钟期限。
离线构建运行 `../scripts/build_search.sh`，需 C++17、PCL common/kdtree、
Eigen3、Python 开发头、NumPy、pybind11；RTAB-Map 导出器另需 0.23.7。
标准 `../scripts/build.sh` 已使用服务虚拟环境 Python 调用原生构建，
CTest 同时检查真实模块加载和取消接口；`../scripts/start.sh` 在注册
Atlas 前检查扩展来自本包构建目录并可加载。此检查不启用生产重定位。
`relocalization_rbnx.search.load_search_index` 和
`load_planar_registration` 从同一份摘要校验的索引加载栅格及点云。
内部 `computation.compute_candidate` 已串联 ROS 采集和本扩展，匹配后复查
计算期间静止、TF、时钟及新鲜度。内部后台任务调度、候选保存和失效入口
已实现；运行服务尚未接入此链路，高度/倾角来源和位姿交接仍待实现。
Python `matching.OfflineMatcher` 已串联两种计算与独立窗口评分、栅格
矛盾检查；仅捕获 `RegistrationRejected` 几何拒绝，计算故障及中断继续
向外传播。共享 token 可查询取消状态，各阶段使用整轮剩余期限。
具体算法与数据一致性要求见部署根目录的 relocalization-plan.md。
