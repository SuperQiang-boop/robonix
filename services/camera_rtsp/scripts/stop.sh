#!/usr/bin/env bash
set -euo pipefail
# Driver SHUTDOWN releases owned resources. rbnx then terminates this package's
# process group, including FFmpeg/MediaMTX, even if the Driver cannot respond.
# Do not pkill by module name: another deployment may use the same package.
exit 0
