#!/usr/bin/env bash
set -euo pipefail

PKG="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)}"
BUILD_ROOT="$PKG/rbnx-build"
DEPLOY_ROOT="${ROBONIX_DEPLOY_DIR:-$(cd "$PKG/../.." && pwd)}"
SDK2_DIR="${UNITREE_SDK2_DIR:-$DEPLOY_ROOT/third_party/unitree_sdk2}"

cd "$PKG"
rbnx codegen -p "$PKG"

if [ ! -f "$SDK2_DIR/CMakeLists.txt" ]; then
  echo "[g1_arm/build] UNITREE_SDK2_DIR is not an official SDK2 checkout" >&2
  exit 2
fi

cmake -S "$PKG" -B "$BUILD_ROOT/sdk" \
  -DUNITREE_SDK2_DIR="$SDK2_DIR" \
  -DBUILD_EXAMPLES=OFF \
  -DCMAKE_INSTALL_PREFIX="$BUILD_ROOT/sdk/install"
cmake --build "$BUILD_ROOT/sdk" --parallel --target g1_arm_action_cli
cmake --install "$BUILD_ROOT/sdk"

if [ ! -x "$BUILD_ROOT/sdk/install/bin/g1_arm_action_cli" ]; then
  echo "[g1_arm/build] action helper output is missing" >&2
  exit 2
fi

ROBONIX_API_ROOT="$(rbnx path robonix-api)"
PYTHONPATH="$PKG:$ROBONIX_API_ROOT:$BUILD_ROOT/codegen/proto_gen:${PYTHONPATH:-}" \
  python3 -m py_compile "$PKG/g1_arm/main.py"

