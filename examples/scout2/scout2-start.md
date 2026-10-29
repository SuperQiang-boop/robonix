# Scout2 统一启动

建图和重定位使用同一个 `robonix_manifest.yaml`。`map_mode: auto` 在没有已保存地图时进入建图模式；有已保存地图时选择最新地图进入 localization。首次建图期间，重定位服务正常启动并等待，不准备地图索引、不自动发起请求，也不会阻止 mapping 实时建图。定位模式下默认自动请求重定位，结果可通过 `get_status` 查询。

```bash
cd /data/robonix/examples/scout2
source .venv/bin/activate
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=42
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
export RBNX_CODEGEN_PYTHON="$PWD/rbnx-boot/python-codegen-venv/bin/python"
rbnx build -f robonix_manifest.yaml --no-update-check
rbnx boot -f robonix_manifest.yaml --no-update-check
```

`robonix_manifest.yaml` 中重定位服务的 `auto_relocalize` 默认是 `true`：localization 模式会自动请求重定位；改为 `false` 后服务等待手动 `relocalize_by_map(map_id)` 请求。mapping 的地图 ID 由 `map_mode: auto` 从最新保存地图选择；仅地图库为空时使用 `scout2_main` 作为首次建图 ID。Scene 从 mapping 的 lifecycle 广播获取实际 map ID。

启动后通过重定位 `get_status` 检查状态。只有 `localization_valid=true` 才表示重定位成功；`active=true` 只表示服务已启动。定位未通过时 navigation 拒绝自主导航。首次建图保持其他用户已开启的 `can0`，mapping 会按 30 秒周期自动保存，并在正常关闭时再保存一次。

若 Pilot 配置需要 VLM 凭据，应先在当前 shell 中设置 `VLM_API_KEY`，不要把密钥写入 manifest 或此文件。
