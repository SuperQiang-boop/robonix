# G1 face camera RGB primitive

This package bridges the Unitree G1 face camera's existing ROS 2 RGB topic into
the Robonix camera capability namespace. It does not start or configure a
camera driver.

## Capability surface

| Contract | Mode | Transport | Endpoint |
| --- | --- | --- | --- |
| `robonix/primitive/camera/driver` | rpc | gRPC | lifecycle driver |
| `robonix/primitive/camera/rgb` | topic_out | ROS 2 | `/face/camera/color/image_raw` |

The deployment instance name is `face_camera`, so consumers select this RGB
provider with `provider_id: face_camera` when more than one camera is present.

## Configuration

```yaml
- name: face_camera
  path: ${ROBONIX_DEPLOY_DIR}/primitives/primitive-unitree-g1-face-camera-rgb-rbnx
  config:
    rgb_topic: /face/camera/color/image_raw
    sentinel_timeout_s: 30.0
```

Build and test with:

```bash
rbnx build -p .
python3 -m py_compile face_camera/main.py
```
