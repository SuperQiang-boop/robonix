#!/usr/bin/env bash
set -euo pipefail
PKG="$(cd "$(dirname "$0")/.." && pwd)"
export PYTHONPATH="$PKG:$PKG/rbnx-build/codegen/proto_gen:${PYTHONPATH:-}"
PYTHON="${CAMERA_RTSP_TEST_PYTHON:-$PKG/rbnx-build/venv/bin/python}"
"$PYTHON" -m unittest discover -s "$PKG/tests" -v
"$PYTHON" - <<'PY'
import camera_rtsp_pb2 as pb
from camera_rtsp import main
class Context:
    def abort(self, code, detail):
        raise RuntimeError(detail)
try:
    main.get_stream(pb.GetStream_Request(), Context())
except RuntimeError as exc:
    assert 'not active' in str(exc)
else:
    raise AssertionError('inactive request succeeded')
assert pb.GetStream_Response.FromString(pb.GetStream_Response(available=True).SerializeToString()).available
print('protobuf and inactive gate OK')
PY
