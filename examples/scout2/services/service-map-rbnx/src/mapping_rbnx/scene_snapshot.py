# SPDX-License-Identifier: MulanPSL-2.0
"""Save Scene objects after a successful automatic spatial checkpoint."""
from __future__ import annotations

import http.client
import json
import logging
import os
import urllib.error
import urllib.request

log = logging.getLogger(__name__)


def save_objects(map_id: str, generation: int) -> dict:
    """Notify Scene without calling mapping's save RPC again.

    The bounded request runs in the checkpoint thread, leaving SLAM online.
    Failure preserves the spatial save and is logged; the next checkpoint
    retries both saves. Scene checks the live map and generation before
    committing its snapshot.
    """
    url = os.environ.get("SCENE_WEB_URL", "http://127.0.0.1:50107").rstrip("/")
    try:
        request = urllib.request.Request(
            f"{url}/api/maps/save",
            data=json.dumps({
                "map_id": map_id, "generation": generation, "spatial_saved": True,
            }).encode("utf-8"),
            headers={"Content-Type": "application/json"},
            method="POST",
        )
        with urllib.request.urlopen(request, timeout=10.0) as response:
            result = json.loads(response.read().decode("utf-8"))
        if not isinstance(result, dict):
            raise ValueError("Scene returned a non-object response")
        if not result.get("ok") or result.get("map_id") != map_id:
            raise ValueError(result.get("detail") or "Scene did not confirm this map's snapshot")
        if not result.get("snapshot_partition"):
            raise ValueError("Scene did not commit an object snapshot")
    except urllib.error.HTTPError as exc:
        detail = exc.read(4096).decode("utf-8", errors="replace")
        result = {"ok": False, "detail": f"HTTP {exc.code}: {detail}"}
    except (OSError, ValueError, TypeError, http.client.HTTPException) as exc:
        result = {"ok": False, "detail": str(exc)}
    if result.get("ok"):
        log.info("checkpoint objects saved: map_id=%s generation=%s objects=%s partition=%s",
                 map_id, generation, result.get("objects"), result["snapshot_partition"])
    else:
        log.warning("spatial checkpoint saved, but Scene objects were not saved: map_id=%s generation=%s: %s",
                    map_id, generation, result.get("detail"))
    return result
