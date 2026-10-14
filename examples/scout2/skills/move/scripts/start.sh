#!/usr/bin/env bash
# SPDX-License-Identifier: MulanPSL-2.0
# move skill start (native). Runs the atlas bridge under the selected Python environment,
# with ROS + codegen + robonix-api on PYTHONPATH.
set -euo pipefail
PKG="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"
cd "$PKG"

# ROS's setup.bash references unset vars; under `set -u` that is
# a FATAL exit. Relax `set -u` around the sources.
# shellcheck disable=SC1091
set +u
source /opt/ros/humble/setup.bash 2>/dev/null || true
source "$HOME/.cargo/env" 2>/dev/null || export PATH="$HOME/.cargo/bin:$PATH"
set -u

# Prefer an activated Python environment; otherwise locate Conda.
# An activated virtual environment supplies the runtime without Conda.
if [[ -n "${VIRTUAL_ENV:-}" ]]; then
    CONDA_PY="$VIRTUAL_ENV/bin/python3"
    if [[ ! -x "$CONDA_PY" ]]; then
        echo "error: VIRTUAL_ENV does not contain an executable bin/python3" >&2
        exit 1
    fi
else
    CONDA="${CONDA_EXE:-}"
    if [[ -z "$CONDA" ]]; then
        CONDA="$(type -P conda || true)"
    fi
    for candidate in "$HOME/miniforge3/bin/conda" "$HOME/miniconda3/bin/conda" "$HOME/anaconda3/bin/conda" /opt/conda/bin/conda /opt/miniconda3/bin/conda; do
        [[ -n "$CONDA" ]] && break
        [[ -x "$candidate" ]] && CONDA="$candidate" && break
    done
    if [[ ! -x "$CONDA" ]]; then
        echo "[move] error: could not find conda binary; activate a Python virtual environment or Conda, or set CONDA_EXE."
        exit 1
    fi

    # Use Python from the named Conda environment
    CONDA_ENV="robonix-scout2"
    if ! CONDA_PY="$("$CONDA" run -n "$CONDA_ENV" python3 -c 'import sys; print(sys.executable)')" || [[ -z "${CONDA_PY:-}" ]]; then
        echo "[move] error: failed to get Python from conda env '$CONDA_ENV'."
        exit 1
    fi
fi

# Set PYTHONPATH: codegen + robonix-api
export PYTHONPATH="$PKG:$PKG/rbnx-build/codegen/proto_gen:$PKG/rbnx-build/codegen/robonix_mcp_types:${PYTHONPATH:-}"
if ROBONIX_PY="$(rbnx path robonix-api 2>/dev/null)"; then
    export PYTHONPATH="$ROBONIX_PY:$PYTHONPATH"
fi

# -u: unbuffered stdio so log lines flush immediately
exec "$CONDA_PY" -u -m move_skill.atlas_bridge
