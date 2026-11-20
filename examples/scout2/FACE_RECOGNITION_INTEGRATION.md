# VSIS 人脸识别 service 接入记录

## 1. 外部项目盘点

外部项目已复制到 service 包内：`services/face_recognition/vsis_video_service/`。原始项目目录仍可作为外部覆盖目录使用。

- 启动：`bin/vsis_video_service`，工作目录必须是外部项目根目录。
- 输入：`config/cfg.yaml` 中的 RTSP/V4L2 视频源。
- 算法：InspireFace，使用 `person/db` 特征库和 `person/mapping.json` 名单映射。
- 行为：连续处理视频帧，识别到名单人员后按 `confirm_hits`、`absent_sec` 和 `greet_cooldown_sec` 控制问候。
- 输出：向 `audio.host:audio.port/speak` 发 HTTP JSON 请求，正文为 `{ "text": "..." }`。
- 就绪条件：日志出现 `特征库就绪`，表示 InspireFace 已启动、特征库已加载且名单已读取。
- 停止：SIGTERM；bridge 等待退出，超时后 SIGKILL。

## 2. Robonix 边界

该功能承担持续的人脸识别，接入为 `service`。外部 C++ 程序保留算法和设备处理；Robonix bridge 负责 provider 注册、Driver lifecycle、Atlas 依赖发现、语音协议转换、ready gate 和资源清理。

新增包：`services/face_recognition`。

- provider id：`face_recognition`
- namespace：`robonix/service/face_recognition`
- capability：`robonix/service/face_recognition/driver`
- lifecycle IDL：复用 `lifecycle/srv/Driver.srv`
- 上游依赖：按 Atlas 查询 `robonix/service/speech/speak`，transport 为 MCP，可选 `speech_provider_id` 约束 provider。

没有新增业务 RPC；外部程序的问候行为继续由其现有配置控制。

## 3. 生命周期流程

```text
start.sh
  -> bridge 注册 Atlas 和 Driver
  -> CMD_INIT
       校验 external_root、binary_path、端口、超时
       查询 speech/speak，保存冷配置
  -> CMD_ACTIVATE
       重新查询 speech/speak
       建立 Atlas channel
       启动 127.0.0.1:10086/speak 兼容代理
       启动 VSIS 子进程
       等待“特征库就绪”或超时/异常退出
       -> ACTIVE
  -> VSIS 发送 /speak
       bridge 调用 speech MCP speak
       返回 JSON 结果
  -> CMD_DEACTIVATE / CMD_SHUTDOWN
       停止 VSIS
       关闭 HTTP 代理
       关闭 Atlas channel
```

启动失败会按相反顺序清理已申请的资源，并返回 `Err`。deactivate/shutdown 可重复执行。

## 4. 部署配置

`robonix_manifest.yaml` 已按依赖顺序启用 `speech`，随后启用 `face_recognition`。实例配置通过 Driver INIT 传入：

```yaml
- name: face_recognition
  path: ./services/face_recognition
  config:
    # 省略 external_root 和 binary_path，使用包内 vsis_video_service/
    startup_timeout: 45
    speak_host: 127.0.0.1
    speak_port: 10086
```

外部 `config/cfg.yaml` 的 `audio.host` 和 `audio.port` 必须与上述值相同。必须先确保 speech service 和 audio primitive 已可用。

## 5. 构建与验收

```bash
cd /home/szh/robonix/examples/scout2/services/face_recognition
bash scripts/build.sh
bash scripts/smoke.sh
```

单包运行：

```bash
rbnx start -p /home/szh/robonix/examples/scout2/services/face_recognition
```

完整部署：

```bash
rbnx boot -f /home/szh/robonix/examples/scout2/robonix_manifest.yaml
```

验收时检查：

1. Atlas 中出现 `robonix/service/face_recognition/driver`，kind 为 service。
2. `CMD_INIT` 能拒绝不存在的外部目录、不可执行二进制、非法端口和未知配置键。
3. `CMD_ACTIVATE` 在 speech capability 不存在时失败，不会启动孤立的 VSIS 进程。
4. VSIS 未输出 `特征库就绪` 或提前退出时不会进入 ACTIVE。
5. VSIS 识别到人员后，`/speak` 请求经 bridge 到达 speech service 并实际播放。
6. 连续执行 activate/deactivate/activate 后无残留子进程、HTTP 线程或 Atlas channel。
7. shutdown 后确认 `vsis_video_service` 进程已退出。

当前已完成：文件结构、manifest、配置约束、bridge 静态语法、配置单元测试。真实构建、Atlas 启动、摄像头识别和语音播放需要目标机上的 Robonix 依赖、摄像头、InspireFace 运行库、特征库和音频设备，不能用静态测试替代。

## 6. 从零复现

以下顺序可直接用于另一个外部 C++ 项目。先完成文件和 contract，再编写 bridge，最后才加入部署清单。

### 6.1 复制项目

执行：

    cd /home/szh/robonix/examples/scout2
    mkdir -p services/face_recognition/{face_recognition,capabilities,scripts,tests}
    mkdir -p services/face_recognition/vsis_video_service
    rsync -a --exclude='build/' --exclude='face_enroll/build/' --exclude='logs/' \
      /home/szh/vsis_video_service/ services/face_recognition/vsis_video_service/

必须保留 bin/vsis_video_service、bin/libInspireFace.so、config/cfg.yaml、person/db、person/mapping.json、照片、模型 pack 和 third_party/inspireface_sdk。不要复制 build/ 和运行日志。

### 6.2 创建 package 文件

package_manifest.yaml 必须包含包身份、build/start/stop 和所有 capability：

    manifestVersion: 1
    package:
      name: com.robonix.service.face_recognition
      version: 0.1.0
      vendor: robonix
      license: MulanPSL-2.0
    build: bash scripts/build.sh
    start: bash scripts/start.sh
    stop: bash scripts/stop.sh
    capabilities:
      - name: robonix/service/face_recognition/driver

创建 capabilities/driver.v1.toml：

    [contract]
    id = "robonix/service/face_recognition/driver"
    version = "1"
    kind = "service"
    idl = "lifecycle/srv/Driver.srv"
    description = "Lifecycle control interface for the VSIS face recognition service."

    [mode]
    type = "rpc"

Driver contract 复用全局 lifecycle contract，不手工生成 protobuf。当前接入不新增业务 RPC。

pyproject.toml 需要依赖 robonix-api、声明 Python 3.10+ 和 face_recognition* 包；创建空的 face_recognition/__init__.py。

### 6.3 配置约束

config.spec 和 Settings.parse 必须保持一致：

    external_root: 可选，默认包内 vsis_video_service
    binary_path: 可选，默认 external_root/bin/vsis_video_service
    startup_timeout: 1 到 300 秒，默认 45
    speak_host: 默认 127.0.0.1
    speak_port: 1 到 65535，默认 10086
    speech_provider_id: 可选的 speech provider id
    ready_marker: 非空，默认“特征库就绪”

未知键、空路径、不可执行二进制、非法端口和非法超时必须在 INIT 返回 Err。包内默认路径由 main.py 计算为：

    bundled_root = Path(__file__).resolve().parent.parent / "vsis_video_service"

### 6.4 编写 bridge

main.py 必须创建：

    service = Service(
        id="face_recognition",
        namespace="robonix/service/face_recognition",
    )

id 必须和部署清单 name 完全相同。speech 依赖按 contract 和 MCP transport 查询：

    SPEAK = "robonix/service/speech/speak"

    def _find_speech():
        query = {"contract_id": SPEAK, "transport": Transport.MCP}
        if _settings and _settings.speech_provider_id:
            query["provider_id"] = _settings.speech_provider_id
        return ATLAS.find_unique_capability(**query)

HTTP 兼容代理监听 speak_host:speak_port。VSIS 发出的 POST /speak JSON 文本转换为：

    await client.call_tool("speak", {"target": "", "text": text})

MCP 调用放在独立事件循环，并限制约 60 秒。非法输入返回 400，上游错误返回 502，超时返回 504；BrokenPipeError 只记录 caller 已断开，不重复写响应，也不自动重试。

### 6.5 生命周期和 ready gate

INIT 只解析冷配置并探测 speech；ACTIVATE 才创建 Atlas channel、启动代理和启动 VSIS：

    subprocess.Popen(
        [str(settings.binary_path)],
        cwd=settings.external_root,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
    )

stdout drain 线程逐行查找 ready_marker。进程提前退出或超时都返回 Err；仅 Popen 成功不能进入 ACTIVE。activate 任一步失败都必须清理已申请资源。deactivate/shutdown 要覆盖部分初始化状态并能重复执行。

## 7. 外部 C++ 改动和构建

原项目保留算法和摄像头处理。FaceRecognitionEngine::hello() 继续发送：

    json body = {{"text", text}};
    cli.Post("/speak", body.dump(), "application/json");

VSIS 的 audio.host/audio.port 必须和 bridge 的 speak_host/speak_port 相同。bridge 等待 speech 最多 60 秒时，C++ 读取超时必须更长：

    cli.set_connection_timeout(1, 0);
    cli.set_read_timeout(65, 0);

否则 speech/audio_driver 可能已经播放，VSIS 却提前报告语音服务不可达。修改 C++ 后必须重新编译：

    cd services/face_recognition
    cmake -S vsis_video_service -B rbnx-build/vsis -DCMAKE_BUILD_TYPE=Release
    cmake --build rbnx-build/vsis --parallel 2

强制全量重建：

    rm -rf rbnx-build/vsis
    bash scripts/build.sh
    stat vsis_video_service/bin/vsis_video_service

## 8. build/start/stop 脚本

build.sh 必须依次完成 uv 环境、robonix codegen 和 CMake：

    uv venv --python /usr/bin/python3 --system-site-packages $PKG/rbnx-build/venv
    uv pip install --python $PKG/rbnx-build/venv/bin/python $SOURCE/pylib/robonix-api $PKG
    RBNX_CODEGEN_PYTHON=$PKG/rbnx-build/venv/bin/python rbnx codegen -p $PKG
    cmake -S $PKG/vsis_video_service -B $PKG/rbnx-build/vsis -DCMAKE_BUILD_TYPE=Release
    cmake --build $PKG/rbnx-build/vsis --parallel 2

实际脚本应使用 RBNX_PACKAGE_ROOT 和 ROBONIX_SOURCE_PATH 计算绝对路径。start.sh 设置 package 和 rbnx-build/codegen/proto_gen 的 PYTHONPATH，执行 python -m face_recognition.main。stop.sh 只做幂等兜底，不使用宽泛 pkill。

## 9. 部署顺序和播放链路

依赖顺序必须是 audio primitive、speech service、face_recognition service：

    primitive:
      - name: audio_driver
        # audio primitive 的实际 manifest 和声卡配置
    service:
      - name: speech
        path: ROBONIX_SOURCE_PATH/services/speech
        config:
          default_speaker_provider_id: audio_driver
      - name: face_recognition
        path: ./services/face_recognition
        config:
          startup_timeout: 45
          speak_host: 127.0.0.1
          speak_port: 10086

speech/speak 会查询 robonix/primitive/audio/speaker，然后通过 gRPC stream 调 audio_driver。face service 不直接操作声卡，也不需要 audio primitive 端口。

## 10. 四层验收

配置、语法和单测：

    cd services/face_recognition
    python3 -m py_compile face_recognition/main.py
    python3 -m unittest discover -s tests -v

包构建：

    bash scripts/build.sh
    test -x vsis_video_service/bin/vsis_video_service
    find rbnx-build/codegen -type f | sort

单包 lifecycle：

    rbnx start -p /home/szh/robonix/examples/scout2/services/face_recognition

完整部署：

    rbnx boot -f /home/szh/robonix/examples/scout2/robonix_manifest.yaml

必须验证：Atlas 注册正确；speech 缺失时 activate 失败且无孤立进程；ready marker 缺失时不进入 ACTIVE；activate/deactivate/activate 无泄漏；shutdown 后 VSIS 退出；识别后请求经 speech MCP 到达 audio_driver；audio_driver 日志或实体声卡确认播放。MCP HTTP 200 只证明 speech 请求被接受。

## 11. 故障排查

语音服务不可达 127.0.0.1:10086：检查 VSIS YAML 与部署 config 的 host/port，确认运行的是最新 C++ 二进制。若 MCP 已 200 而 VSIS 仍在 3 秒报错，说明旧二进制仍在运行，重新 CMake build 并检查 stat mtime。

BrokenPipeError：说明 caller 已因旧超时断开；确认日志中的 main.py 路径属于当前 checkout，当前 bridge 应只记录断开，不发送第二份响应。

识别成功但无声音：检查 speech MCP、default_speaker_provider_id、Atlas 的 audio/speaker capability、audio_driver 声卡权限和播放日志。

特征库为空：检查 person/db、mapping、照片、InspireFace pack 和工作目录；mapping 中的 ID 必须和特征库 identity 对应。

rbnx build 没有重建 C++：确认目标包 build.sh 含 CMake 两行；删除 rbnx-build/vsis 后重建。rbnx 不会自动猜测外部 C++ 项目的构建命令。

## 12. 交付文件清单

services/face_recognition 至少包含：

    package_manifest.yaml
    pyproject.toml
    config.spec
    CAPABILITY.md
    README.md
    capabilities/driver.v1.toml
    face_recognition/__init__.py
    face_recognition/main.py
    scripts/build.sh
    scripts/start.sh
    scripts/stop.sh
    scripts/smoke.sh
    tests/test_config.py
    tests/test_speech_response.py
    vsis_video_service/bin/vsis_video_service
    vsis_video_service/bin/libInspireFace.so
    vsis_video_service/config/cfg.yaml
    vsis_video_service/person/db/
    vsis_video_service/person/mapping.json
    vsis_video_service/person/photo/
    vsis_video_service/third_party/inspireface_sdk/

rbnx-build 是构建产物，生成的 protobuf/MCP 类型不能手工修改。真实 Atlas、摄像头和 audio_driver 验收必须在目标设备执行。
