#!/usr/bin/env bash
set -euo pipefail

PKG="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)}"
DEPLOY="$(cd "$PKG/../.." && pwd)"
SDK_ROOT="$DEPLOY/third_party/unitree_sdk2"
BUILD_DIR="$PKG/rbnx-build"

if [[ ! -f "$SDK_ROOT/CMakeLists.txt" ]]; then
  echo "Unitree SDK2 submodule not found at $SDK_ROOT" >&2
  echo "Initialize it with: git submodule update --init third_party/unitree_sdk2" >&2
  exit 2
fi

if [[ "${RBNX_BUILD_CLEAN:-}" == "1" ]]; then
  rm -rf "$BUILD_DIR"
fi
mkdir -p "$BUILD_DIR"

cmake -S "$PKG" -B "$BUILD_DIR/native" \
  -DUNITREE_SDK_ROOT="$SDK_ROOT" \
  -DCMAKE_BUILD_TYPE=Release
cmake --build "$BUILD_DIR/native" --target g1_audio_sdk_helper --parallel
mkdir -p "$BUILD_DIR/bin"
cp "$BUILD_DIR/native/g1_audio_sdk_helper" "$BUILD_DIR/bin/g1_audio_sdk_helper"
SDK_LIB_DIR="$SDK_ROOT/thirdparty/lib/$(uname -m)"
mkdir -p "$BUILD_DIR/lib"
for library in libddsc.so libddsc.so.0 libddscxx.so libddscxx.so.0; do
  if [[ ! -f "$SDK_LIB_DIR/$library" ]]; then
    echo "Required Unitree SDK2 runtime library not found: $SDK_LIB_DIR/$library" >&2
    exit 2
  fi
  cp -L "$SDK_LIB_DIR/$library" "$BUILD_DIR/lib/$library"
done
mkdir -p "$BUILD_DIR/share/licenses/unitree_sdk2"
cp "$SDK_ROOT/LICENSE" "$BUILD_DIR/share/licenses/unitree_sdk2/"
cp -R "$SDK_ROOT/licenses" "$BUILD_DIR/share/licenses/unitree_sdk2/"

ROBONIX_API="$(rbnx path robonix-api)"
rbnx codegen -p "$PKG" --ros2 --out-dir "$BUILD_DIR/codegen"
PYTHONPATH="$PKG:$ROBONIX_API:$BUILD_DIR/codegen/proto_gen:${PYTHONPATH:-}" \
  python3 -c 'import g1_audio.main'
PYTHONPATH="$PKG:$ROBONIX_API:$BUILD_DIR/codegen/proto_gen:${PYTHONPATH:-}" \
  python3 -m py_compile "$PKG/g1_audio/main.py"

echo "[g1_audio] build complete"
