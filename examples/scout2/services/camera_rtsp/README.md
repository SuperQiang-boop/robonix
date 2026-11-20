# Camera RTSP service

公共 service 将 Atlas 中相机原语的 ROS 2 RGB 图像转换为 H.264 RTSP 流。
默认自主管理 MediaMTX 服务端与 FFmpeg；也可通过 `manage_server: false`
向外部 RTSP 服务端推流。不依赖 scene，不直接占用相机设备。

## 安装和构建

Linux 主机需安装与 Python ABI 匹配的 ROS 2、rclpy、sensor_msgs，FFmpeg
（含 libx264），以及 MediaMTX 1.x（可执行文件名 `mediamtx`）。外部服务端模式
无需本机 MediaMTX。系统依赖需预装；启动过程不下载文件。

```bash
# 例如 ROS 2 Humble 使用 /usr/bin/python3 (3.10)
source /opt/ros/humble/setup.bash
bash scripts/build.sh
bash scripts/smoke.sh
```

构建使用 uv 创建带系统包的 `rbnx-build/venv`，安装项目与源码树中的
robonix-api，再执行 codegen。需 uv、rbnx、robonix-codegen 可用。
环境变量：`ROBONIX_SOURCE_PATH` 指向源码根（默认由 git 查找）；
`CAMERA_RTSP_PYTHON` 选择与 ROS 匹配的 Python（默认 /usr/bin/python3）；
`CAMERA_RTSP_ROS_SETUP` 可指定启动时 source 的 ROS setup.bash；
`ROBONIX_ATLAS` 默认 127.0.0.1:50051；`ROS_DOMAIN_ID`、RMW 配置须与相机一致。

## 部署和播放

将 deployment.example.yaml 的条目加入部署清单的 service 列表，相机需先启动，
同一 Atlas 中实例名称为 camera_rtsp。所有业务参数见 config.spec。
默认管理模式仅监听 loopback，播放地址示例 `rtsp://127.0.0.1:8554/camera`。
跨机器播放时使用外部 MediaMTX，在其配置中自行指定监听、路径与访问控制，
设置 manage_server=false 并填写消费者可达的 RTSP URL。

```bash
ffplay -rtsp_transport tcp rtsp://127.0.0.1:8554/camera
```

支持 rgb8/bgr8/rgba8/bgra8/mono8，保留图像内容并去除行填充；奇数尺寸编码时
补齐为偶数。仅支持 ROS 2 Image，不支持 JPEG 或 gRPC 相机输入。
采用 sensor-data best-effort QoS；输出最多为 fps，慢相机不补帧。
媒体时间戳由 FFmpeg 生成，不保留 ROS header 时间戳或 frame_id。
收到首帧且 RTSP 解码成功才激活；无帧、格式变化、进程退出和管道阻塞明确报错。
底层故障停止编码和服务端，GetStream 返回不可用；通过 deactivate/activate 恢复。

## 验收

scripts/smoke.sh 检查配置、行步长/颜色转换、生成的 protobuf 与不活动调用门禁。
实际设备验收需运行 Atlas、相机和本服务，调用 Driver INIT/ACTIVATE 后以 ffplay
播放；重复 DEACTIVATE/ACTIVATE，停止相机确认不可用，再 SHUTDOWN 检查端口释放。
stop.sh 依赖 rbnx 在 Driver shutdown 后终止包进程组，避免按名字误杀其他实例。
不通过 rbnx 启动时应对主进程发送 SIGTERM，不能仅执行 stop.sh。
