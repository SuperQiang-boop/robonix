# 重定位服务

## 当前实现

重定位服务根据当前已加载地图和实时雷达、里程计数据估计小车在地图坐标系中的平面位姿。服务完成全局匹配后，将估计位姿交给 mapping，并用之后采集的独立雷达扫描验证；只有验证通过且持续检查仍有效时，`localization_valid` 才为 `true`。`active` 仅表示服务可用，不表示定位成功。

服务不控制底盘运动。定位无效时，navigation 拒绝导航任务；定位失效或地图重新加载时，旧定位结果不能继续作为导航依据。

## 自动与手动模式

建图和定位使用同一份 `robonix_manifest.yaml`；配置入口是重定位服务的 config：

```yaml
# robonix_manifest.yaml 的 service 列表内
- name: relocalization
  path: ./services/service-relocalization
  config:
    # 其余传感器、标定、路径配置沿用完整清单
    auto_relocalize: true
```

`auto_relocalize` 默认 `true`。mapping 的 `map_mode: auto` 决定实际启动模式：
无保存地图时开始实时建图并按 30 秒周期自动保存，重定位服务只等待，不生成索引或自动请求。
有保存地图时选最新保存地图进入 localization；完成数据库加载与占据图检查后，
服务读取实际加载的地图身份，准备匹配索引并发起重定位。每次加载最多自动尝试 3 次，
不会一直搜索。保存图不会使本次建图自动转入 localization。
索引准备遇到暂时故障时会退避重试，间隔最长 30 秒；这不计入自动定位请求的 3 次限制。

将 `auto_relocalize` 改为 `false` 并重启，服务只监控地图、发布状态，等待手动请求。
配置修改不需重新编译服务。开关放在 manifest，不从标定 profile 读取。
手动触发前也必须已加载定位地图；mapping 模式的请求会被拒绝，不会影响继续建图。
正常调用完成后只持续检查已有定位，不等同于反复发起全局重定位。

## 接口

接口属于 `robonix/service/relocalization`：

| contract | 请求 | 返回 / 用途 |
| --- | --- | --- |
| `relocalize_by_map` | `map_id` | `accepted`、`request_id`、`detail`。服务从当前 mapping 已加载快照读取地图 generation 和数据库摘要，并使用服务配置的定位期限。 |
| `get_status` | 空请求 | 当前定位状态，包括请求 ID、有效标志、地图身份、位姿、残差、重叠率及详情。 |
| `cancel` | `request_id` | 取消对应请求；返回 `ok` 和 `detail`。 |

主要状态字段：

- `busy`：定位请求正在处理。
- `localization_valid`：定位及独立新扫描验证通过，且持续验证仍有效。只有为 `true` 时，才能使用位姿和质量字段。
- `x`、`y`、`theta`：地图坐标系下的平面位姿，单位分别为米、米、弧度。
- `residual_m`：匹配残差，单位米；缺少结果时为 `-1`。
- `overlap`：点云重叠比例，范围 `[0, 1]`，不是成功概率。
- `ambiguous`：匹配是否存在歧义。
- `detail`：当前状态或失败原因。
- `map_id`、`generation`、`database_sha256`：本次状态对应的已加载地图身份。
- `observation_stamp_ns`：用于定位的观测采集时间；消费者应检查其持续更新，不能只依赖 `published_stamp_ns`。

## 手动调用示例

以下示例展示调用顺序。`relocalize_by_map` 和 `get_status` 是通过 Atlas 发现并连接对应 contract 后得到的 RPC 方法；其中 `pb` 是该服务生成的 protobuf 模块。

```python
import time

request = pb.RelocalizeByMap_Request(map_id="scout2_main")
accepted = relocalize_by_map(request, timeout=3)
if not accepted.accepted:
    raise RuntimeError(accepted.detail)

request_id = accepted.request_id
deadline = time.monotonic() + 30
while time.monotonic() < deadline:
    status = get_status(pb.GetStatus_Request(), timeout=3).status
    if status.request_id == request_id and not status.busy:
        if not status.localization_valid:
            raise RuntimeError(status.detail or "relocalization failed")
        print({
            "request_id": request_id,
            "map_id": status.map_id,
            "generation": status.generation,
            "x": status.x,
            "y": status.y,
            "theta": status.theta,
            "residual_m": status.residual_m,
            "overlap": status.overlap,
        })
        break
    time.sleep(0.5)
else:
    raise TimeoutError(f"relocalization request {request_id} is still pending")
```

`accepted=true` 只表示请求已受理，不代表定位成功。调用方应保存 `request_id`，直到状态中的请求 ID 与其一致且 `busy=false`；仅 `localization_valid=true` 表示本次定位通过。若失败，查看 `detail` 后由调用方重新提交；手动请求不会自动重试。若需停止正在进行的请求，调用 `cancel` 并传入对应 `request_id`。

`map_id` 必须与当前 mapping 加载的地图 ID 一致。调用方不需要提供 generation、数据库摘要、初始位姿或算法阈值；地图身份及期限由服务解析和配置。服务启动后从 mapping 读取实际加载的地图身份，并在 `data/<map_id>/<key>/` 中查找匹配索引；找不到时，使用已构建的快照导出器为稳定保存的地图生成索引。索引生成失败会通过状态和日志报告，定位保持无效，navigation 不放行。

常规 `rbnx build` 同时构建快照导出器，并记录实际使用的 mapping 镜像 ID。
镜像替换时重新编译和验证导出器；若之后删除该镜像，重新执行统一构建即可恢复，
不需要删除地图或物体数据。索引准备失败时尚未创建重定位请求，`request_id` 可能为空；
此时界面仍可能显示数据库保存的上次位置，应以 `localization_valid` 和 `detail` 判断。

统一启动命令见 [scout2-start.md](../../scout2-start.md)。不需清除历史地图，也不需生成
`robonix_manifest.localization*.yaml`。auto 模式按保存地图目录修改时间选择最新地图，
同时间按 ID 排序；需要指定地图时，使用原清单中的 `map_mode: localization` 与目标 `map_id`。
Scene 从 mapping 的 lifecycle 读取地图身份，不再静态绑定 `scout2_main`。
`scout2_main` 仅作为空图库首次建图的名称；用户保存其他 ID 后可正常选用。

稳定 mapping 模式不会持续发送导航零速度干扰遥控；导航目标及 Nav2 输出仍需有效定位。
切换地图、重载或退出 localization 会撤销旧结果，不能借旧 request_id 恢复导航资格。

## 当前验证与后续验收

2026-10-28：已通过配置与运行时回归，以及隔离 Atlas/ROS 生产入口联调：
无地图的 mapping 等待、有地图自动定位、同图重载后重新定位、关闭自动后手动请求和资源回收。
本轮未启动实车；构建与测试详情见 [IMPLEMENTATION_STATUS.md](IMPLEMENTATION_STATUS.md)。

当前 `scout2_main` 已实际完成索引生成与加载检查：98 个优化节点、1 个连通分量，
外参一致，起点、中段、末段保存扫描的离线几何检查通过。此检查使用保存扫描，
搬放后的实时重定位仍需启动后的独立雷达观测验证。

实车仍需验证搬放后定位、错误位置拒绝、持续有效性和受控短距离导航。
合成数据联调不能代替实车精度、召回率和耗时验收。
