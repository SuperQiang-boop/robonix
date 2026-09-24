#!/usr/bin/env bash
set -euo pipefail

PKG="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)}"
cd "$PKG"
if ! ROBONIX_API="$(rbnx path robonix-api 2>/dev/null)"; then
  echo "rbnx is required to locate robonix-api" >&2
  exit 2
fi

export PYTHONPATH="$ROBONIX_API:$PKG:$PKG/rbnx-build/codegen/proto_gen:${PYTHONPATH:-}"
export G1_AUDIO_SDK_HELPER="${G1_AUDIO_SDK_HELPER:-$PKG/rbnx-build/bin/g1_audio_sdk_helper}"
exec python3 -m g1_audio.main
