#!/usr/bin/env bash
set -euo pipefail
# The bridge owns the VSIS child and terminates it during Driver SHUTDOWN.
# rbnx subsequently terminates the package process group if the bridge is unresponsive.
exit 0
