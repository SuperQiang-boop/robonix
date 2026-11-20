#!/usr/bin/env bash
# Check native Python dependencies, generated modules, and gRPC without starting hardware.
set -euo pipefail
SCOUT2_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
set +u
source "/opt/ros/${ROS_DISTRO:-humble}/setup.bash"
set -u
export PYTHONPATH="$(rbnx path robonix-api):${PYTHONPATH:-}"
python3 - "$SCOUT2_ROOT" <<'PY'
import importlib
from pathlib import Path
import subprocess
import sys

root = Path(sys.argv[1])
for module in ('anyio', 'grpc', 'mcp.server.fastmcp', 'numpy', 'PIL.Image', 'rclpy', 'robonix_api'):
    importlib.import_module(module)
# Each package's stubs must be imported in a fresh interpreter to avoid name collisions.
for name in ('scout2_camera', 'scout2_lidar', 'primitive-robot-description-rbnx', 'primitive-agilex-ranger_mini_v3-chassis-rbnx'):
    proto = root / 'primitives' / name / 'rbnx-build/codegen/proto_gen'
    if not proto.is_dir():
        raise RuntimeError(f'{name}: missing generated modules; build the package first')
    modules = sorted(p.stem for p in proto.glob('*_pb2*.py'))
    subprocess.run(
        [sys.executable, '-c', 'import importlib, sys; sys.path.insert(0, sys.argv[1]); '
         '[importlib.import_module(name) for name in sys.argv[2:]]', str(proto), *modules],
        check=True, timeout=20,
    )
subprocess.run(
    [sys.executable, '-c', 'import grpc; channel = grpc.insecure_channel("127.0.0.1:50051"); channel.close()'],
    check=True, timeout=10,
)
print(f'Python environment OK: {sys.executable}')
PY
