#!/usr/bin/env bash
# SPDX-License-Identifier: MulanPSL-2.0
# Generate Explore's Python protobuf stubs with versions compatible with the
# packages installed in the runtime image.  Generated gRPC modules enforce the
# grpcio-tools version that created them, so host-global Python packages must
# not decide the package's runtime requirement.
set -euo pipefail

if [[ "$#" -lt 1 ]]; then
    echo "usage: $0 <package-root> [rbnx-codegen-args...]" >&2
    exit 2
fi

PKG="$(cd "$1" && pwd)"
shift
VENV="$PKG/rbnx-build/python-codegen-venv"
PROTOBUF_VERSION="6.33.6"
GRPC_TOOLS_VERSION="1.76.0"
GRPCIO_VERSION="1.80.0"

if ! command -v uv >/dev/null 2>&1; then
    echo "[explore/codegen] error: 'uv' not found on PATH. Install: https://docs.astral.sh/uv/" >&2
    exit 1
fi

compatible=0
if [[ -x "$VENV/bin/python" ]]; then
    if "$VENV/bin/python" - "$PROTOBUF_VERSION" "$GRPC_TOOLS_VERSION" "$GRPCIO_VERSION" <<'PY'
from importlib.metadata import version
import sys

expected_protobuf, expected_tools, expected_grpc = sys.argv[1:]
if version("protobuf") != expected_protobuf:
    raise SystemExit(1)
if version("grpcio-tools") != expected_tools:
    raise SystemExit(1)
if version("grpcio") != expected_grpc:
    raise SystemExit(1)
PY
    then
        compatible=1
    fi
fi

if [[ "$compatible" != "1" ]]; then
    echo "[explore/codegen] preparing protobuf $PROTOBUF_VERSION / grpcio-tools $GRPC_TOOLS_VERSION / grpcio $GRPCIO_VERSION"
    rm -rf "$VENV"
    uv venv "$VENV"
    uv pip install --python "$VENV/bin/python" \
        "protobuf==$PROTOBUF_VERSION" \
        "grpcio-tools==$GRPC_TOOLS_VERSION" \
        "grpcio==$GRPCIO_VERSION"
fi

echo "[explore/codegen] rbnx codegen -p $PKG $*"
RBNX_CODEGEN_PYTHON="$VENV/bin/python" \
    PATH="$VENV/bin:$PATH" \
    rbnx codegen -p "$PKG" "$@"

CODEGEN_PYTHONPATH="$PKG/rbnx-build/codegen/proto_gen:$PKG/rbnx-build/codegen/robonix_mcp_types"
PYTHONPATH="$CODEGEN_PYTHONPATH:${PYTHONPATH:-}" \
    "$VENV/bin/python" - "$PROTOBUF_VERSION" "$GRPC_TOOLS_VERSION" "$GRPCIO_VERSION" <<'PY'
from importlib.metadata import version
import sys

expected = {
    "protobuf": sys.argv[1],
    "grpcio-tools": sys.argv[2],
    "grpcio": sys.argv[3],
}
actual = {package: version(package) for package in expected}
if actual != expected:
    raise RuntimeError(f"Explore protobuf environment mismatch: {actual} != {expected}")

import atlas_pb2_grpc  # noqa: F401, E402
import explore_pb2  # noqa: F401, E402
import explore_mcp  # noqa: F401, E402
import robonix_contracts_pb2_grpc  # noqa: F401, E402

# Keep the checked-in IDL, generated protobuf descriptor, and MCP dataclass
# in lockstep. A stale image/codegen previously omitted the frontier diagnostic
# fields, causing status responses to fail during serialization.
status_fields = {
    "rejected_goal_clearance",
    "rejected_route_reachability",
    "rejected_visibility",
    "rejected_view_distance",
    "rejected_local_distance",
    "rejected_near_robot",
    "rejected_cooldown",
    "viewpoint_cells_checked",
}
proto_fields = {
    field.name
    for field in explore_pb2.GetExploreStatus_Response.DESCRIPTOR.fields
}
mcp_fields = set(explore_mcp.GetExploreStatus_Response.__dataclass_fields__)
missing = {
    "protobuf": sorted(status_fields - proto_fields),
    "mcp": sorted(status_fields - mcp_fields),
}
missing = {layer: fields for layer, fields in missing.items() if fields}
if missing:
    raise RuntimeError(f"GetExploreStatus generated fields are stale: {missing}")

probe = explore_pb2.GetExploreStatus_Response(rejected_goal_clearance=1)
if probe.rejected_goal_clearance != 1:
    raise RuntimeError("GetExploreStatus protobuf diagnostic field round-trip failed")

print(
    "[explore/codegen] generated protobuf, gRPC, and MCP imports OK "
    f"(protobuf={actual['protobuf']} grpcio-tools={actual['grpcio-tools']} "
    f"grpcio={actual['grpcio']})"
)
PY
