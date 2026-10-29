#!/usr/bin/env bash
# Compile the exporter natively or in an explicitly selected local image.
set -euo pipefail
PKG="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"
if [[ -n "${RELOCALIZATION_BUILD_IMAGE:-}" ]]; then
    # Resolve the local image to an immutable ID; never silently pull a tag.
    IMAGE="$(docker image inspect "$RELOCALIZATION_BUILD_IMAGE" --format '{{.Id}}')"
    if [[ ! "$IMAGE" =~ ^sha256:[0-9a-f]{64}$ ]]; then
        echo "[relocalization] cannot resolve the local exporter build image" >&2
        exit 1
    fi
    BUILD="$PKG/rbnx-build/backend-container"
    CLEAN=()
    if [[ ! -f "$BUILD/image-id" ]] || [[ "$(cat "$BUILD/image-id")" != "$IMAGE" ]]; then
        # A different runtime can have different headers and libraries even
        # when cached dependency paths and timestamps happen to be unchanged.
        CLEAN=(--clean-first)
    fi
    # An interrupted or failed rebuild must not advertise an old image as
    # compatible with a binary that the build may already have overwritten.
    rm -f "$BUILD/image-id"
    docker run --rm --network none --read-only --tmpfs /tmp \
        --user "$(id -u):$(id -g)" --volume "$PKG:/work" \
        --entrypoint bash "$IMAGE" -c \
        'cmake -S /work/cpp -B /work/rbnx-build/backend-container -DCMAKE_PREFIX_PATH=/opt/ros/humble -DCMAKE_BUILD_TYPE=Release && cmake --build /work/rbnx-build/backend-container --parallel 2 "$@" && ctest --test-dir /work/rbnx-build/backend-container --output-on-failure' \
        build-exporter "${CLEAN[@]}"
    printf '%s\n' "$IMAGE" > "$BUILD/image-id.tmp"
    mv "$BUILD/image-id.tmp" "$BUILD/image-id"
else
    cmake -S "$PKG/cpp" -B "$PKG/rbnx-build/backend" -DCMAKE_BUILD_TYPE=Release
    cmake --build "$PKG/rbnx-build/backend" --parallel 2
    ctest --test-dir "$PKG/rbnx-build/backend" --output-on-failure
fi
