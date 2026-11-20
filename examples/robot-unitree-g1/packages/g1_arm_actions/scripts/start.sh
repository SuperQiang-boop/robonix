#!/usr/bin/env bash
set -euo pipefail

PKG="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ -n "${ROBONIX_SOURCE_PATH:-}" && -d "${ROBONIX_SOURCE_PATH}/pylib/robonix-api/robonix_api" ]]; then
  ROBONIX_API_ROOT="${ROBONIX_SOURCE_PATH}/pylib/robonix-api"
else
  command -v rbnx >/dev/null 2>&1 || {
    echo "rbnx is required to locate robonix-api" >&2
    exit 2
  }
  ROBONIX_API_ROOT="$(rbnx path robonix-api)"
fi

export PYTHONPATH="$PKG:$ROBONIX_API_ROOT:$PKG/rbnx-build/codegen/proto_gen:$PKG/rbnx-build/codegen/robonix_mcp_types:${PYTHONPATH:-}"
exec python3 -m g1_arm_actions.main

