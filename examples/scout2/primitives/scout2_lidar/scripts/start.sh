#!/usr/bin/env bash
# SPDX-License-Identifier: MulanPSL-2.0
# scout2/lidar native runtime — runs directly on the host.
# ROS2 humble drivers (/scan, /velodyne_points) publish on localhost;
# this script bootstraps a Python 3.10 environment with
# ROS2 sys.path + driver code path, then calls Driver(CMD_INIT).
#
# Lifecycle is owned by rbnx/Soma: Driver(CMD_SHUTDOWN), then this
# package's manifest stop hook, then wrapper PGID TERM/KILL.
set -euo pipefail

BOOT="$(cd "$(dirname "$0")" && pwd)/bootstrap.py"

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
          echo "[scout2-lidar] error: could not find conda binary; activate a Python virtual environment or Conda, or set CONDA_EXE."
        exit 1
    fi

    # Use Python from the named Conda environment
    if ! CONDA_PY="$("$CONDA" run -n robonix-scout2 python3 -c 'import sys; print(sys.executable)')" || [[ -z "${CONDA_PY:-}" ]]; then
        echo "[scout2-lidar] error: failed to get Python from conda env 'robonix-scout2'."
        exit 1
    fi
fi

env \
  ROBONIX_ATLAS="${ROBONIX_SIM_ATLAS:-${ROBONIX_ATLAS:-127.0.0.1:50051}}" \
  ROBONIX_ADVERTISE_HOST="${ROBONIX_ADVERTISE_HOST:-127.0.0.1}" \
  "$CONDA_PY" "$BOOT"
