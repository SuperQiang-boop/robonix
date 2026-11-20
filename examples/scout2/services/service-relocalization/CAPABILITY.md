---
description: Global map localization with independent verification and mapping pose handoff.
---

# 重定位 service

provider_id 默认 `relocalization`，namespace 为 `robonix/service/relocalization`。

| contract 后缀 | 传输 | 行为 |
| --- | --- | --- |
| driver | gRPC | INIT / ACTIVATE / DEACTIVATE / SHUTDOWN |
| relocalize | gRPC | 接受 map_id、generation、database_sha256 和期限；立即返回请求 ID；不接受初始位姿 |
| relocalize_by_map | gRPC | 只接受 map_id；从当前 mapping 快照读取 generation 和数据库摘要；立即返回请求 ID |
| get_status | gRPC | 查询受理/计算状态、定位有效性、位姿、残差、重叠和失败原因 |
| cancel | gRPC | 取消指定任务或撤销其已验证结果；最近取消可重复调用 |
| status | ROS 2 topic_out | 按配置周期发布实时状态；消费者必须使过期样本失效 |

服务通过 Atlas 从配置的 provider 连接 lidar3d、底盘 odom、mapping 的
pose / pose_estimate / lifecycle。实际地图副本通过同主机私有协调目录绑定，
不将保存目录最新文件当作已加载数据库。

受理、RPC 注入成功和定位有效分别表达；只有注入后的独立扫描验证通过且持续
验证仍新鲜，`localization_valid` 才为 true。失效时位姿/地图占位字段不可使用。
地图重载、取消、传感器断流、匹配失败、服务停用均撤销本轮导航资格。
本服务不提供 MCP 工具或主动运动控制。
