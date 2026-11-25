# SPDX-License-Identifier: MulanPSL-2.0
"""explore_rbnx atlas bridge — Capability + contract-typed MCP tools.

Tools are typed against the codegen Request/Response dataclasses for the
explore/srv/* contracts (Explore, GetExploreStatus, CancelExplore). The
JSON Schema each MCP tool advertises to the LLM is derived from those
classes via explore.mcp's introspection — no hand-written schemas.
"""
from __future__ import annotations

import logging
import time

from robonix_api import ATLAS, Skill, Ok, Err, Deferred

from .controller import ExploreController
from .configuration import ExploreSettings

logging.basicConfig(level=logging.INFO,
                    format="[explore] %(levelname)s %(message)s")
log = logging.getLogger("explore_rbnx")

explore_skill = Skill(id="explore", namespace="robonix/skill/explore")
ctrl: ExploreController | None = None
settings = ExploreSettings()

# Atlas-resolved inputs the skill consumes. Hard-fail if any required
# input isn't atlas-resolvable: the packaging-spec rule is "no hardcoded
# fallback for cross-package topics", and a skill that can't find its
# dependencies should not pretend to work.
REQUIRED_INPUTS = {
    # (contract_id, transport) — transport must be a concrete enum, not
    # "unspecified", so atlas can return a usable endpoint string. The
    # nav contracts are mode=rpc and simple_nav exposes them as MCP tools
    # (see examples/webots/services/simple_nav/package_manifest.yaml header).
    "map_topic":     ("robonix/service/map/occupancy_grid", "ros2"),
    "nav_navigate":  ("robonix/service/navigation/navigate", "mcp"),
    "nav_status":    ("robonix/service/navigation/navigate/status", "mcp"),
    "nav_cancel":    ("robonix/service/navigation/navigate/cancel", "mcp"),
    "nav_get_speed": ("robonix/service/navigation/get_speed_limit", "mcp"),
}

# Optional inputs: used when available for enhanced features (e.g., glass door
# detection). Skill works without these but with reduced capability.
OPTIONAL_INPUTS = {
    "scene_list_objects": ("robonix/system/scene/list_objects", "mcp"),
}


def resolve_inputs(deadline_s: float = 60.0) -> dict[str, str]:
    resolved: dict[str, str] = {}
    deadline = time.time() + deadline_s
    while time.time() < deadline:
        # Resolve required inputs
        for key, (cid, transport) in REQUIRED_INPUTS.items():
            if key in resolved:
                continue
            try:
                cap_view = ATLAS.find_unique_capability(
                    contract_id=cid, transport=transport,
                )
                ch = explore_skill.connect_capability(cap_view, cid, transport)
            except Exception:  # noqa: BLE001
                continue
            ep = ch.endpoint
            ch.close()
            if ep:
                resolved[key] = ep
                log.info("resolved %s [%s] → %s", cid, transport, ep)
        if len(resolved) == len(REQUIRED_INPUTS):
            break
        time.sleep(2.0)
    else:
        missing = [k for k in REQUIRED_INPUTS if k not in resolved]
        raise RuntimeError(
            f"explore skill cannot find dependencies on atlas: missing "
            f"{[REQUIRED_INPUTS[k][0] for k in missing]}. The skill needs a "
            f"running mapping service (occupancy_grid) and navigation service "
            f"(navigate with status/cancel) before it can start. There is "
            f"intentionally no hardcoded fallback — packaging-spec invariant #1."
        )

    # Resolve optional inputs (best-effort, short timeout)
    optional_deadline = time.time() + 10.0
    for key, (cid, transport) in OPTIONAL_INPUTS.items():
        if key in resolved:
            continue
        try:
            cap_view = ATLAS.find_unique_capability(
                contract_id=cid, transport=transport,
            )
            ch = explore_skill.connect_capability(cap_view, cid, transport)
            ep = ch.endpoint
            ch.close()
            if ep:
                resolved[key] = ep
                log.info("resolved optional %s [%s] → %s", cid, transport, ep)
        except Exception:  # noqa: BLE001
            log.info("optional dependency %s not available, skipping", key)

    return resolved


# ── MCP tools (typed against codegen Request/Response) ──────────────────────
from explore_mcp import (  # noqa: E402
    Explore_Request, Explore_Response,
    GetExploreStatus_Request, GetExploreStatus_Response,
    CancelExplore_Request, CancelExplore_Response,
)


# Map the controller's internal lifecycle words to the canonical async-status
# state names the executor's poller understands (parse_state_name). The
# executor treats unknown values as RUNNING, so the terminal words MUST map:
# without this, `done`/`error` never reached SUCCEEDED/FAILED and the executor
# polled forever.
_CANON_STATE = {
    "idle": "PENDING",
    "exploring": "RUNNING",
    "running": "RUNNING",
    "done": "SUCCEEDED",
    "succeeded": "SUCCEEDED",
    "timeout": "TIMEOUT",
    "canceled": "CANCELED",
    "cancelled": "CANCELED",
    "error": "FAILED",
    "failed": "FAILED",
}


def _canonical_state(state: str) -> str:
    return _CANON_STATE.get(str(state).lower(), str(state).upper())


@explore_skill.mcp("robonix/skill/explore/explore")
def explore(req: Explore_Request) -> Explore_Response:
    """Start exploration; use timeout_s=-1 unless the user specified a duration.
    -1 uses deployment settings, 0 is unlimited, positive values override.
    For Scout2 use max_speed_m_s=0.35 unless a lower ceiling is requested.
    Returns the effective timeout in message; poll status() with run_id."""
    if ctrl is None:
        raise RuntimeError("controller not initialized")
    try:
        handle = ctrl.start(area_hint=req.area_hint,
                            timeout_s=settings.resolve_timeout(req.timeout_s),
                            max_speed_m_s=float(req.max_speed_m_s))
        return Explore_Response(accepted=True, run_id=handle.task_id,
                                message=handle.detail)
    except (RuntimeError, ValueError) as e:
        return Explore_Response(accepted=False, run_id="", message=str(e))


@explore_skill.mcp("robonix/skill/explore/explore/status")
def status(req: GetExploreStatus_Request) -> GetExploreStatus_Response:
    """Poll progress of a running exploration task. Empty run_id = most recent."""
    if ctrl is None:
        raise RuntimeError("controller not initialized")
    s = ctrl.status(req.run_id or None)
    if s is None:
        return GetExploreStatus_Response(
            known=False, state="PENDING", area_m2=0.0, frontiers_left=0,
            elapsed_s=0.0, eta_s=-1.0, detail="no task with that id",
            frontier_regions=0, safe_observation_candidates=0,
            budget_rejected_candidates=0, rejected_goal_clearance=0,
            rejected_route_reachability=0, rejected_visibility=0,
            rejected_view_distance=0, rejected_local_distance=0,
            rejected_near_robot=0, rejected_cooldown=0,
            viewpoint_cells_checked=0, current_target_valid=False,
            current_target_x=0.0, current_target_y=0.0,
        )
    return GetExploreStatus_Response(
        known=True,
        state=_canonical_state(s.get("state", "unknown")),
        area_m2=float(s.get("area_m2", 0.0)),
        frontiers_left=int(s.get("frontiers_left", 0)),
        elapsed_s=float(s.get("elapsed_s", 0.0)),
        eta_s=float(s.get("eta_s", -1.0)),
        detail=str(s.get("detail", "")),
        frontier_regions=int(s.get("frontier_regions", s.get("frontiers_left", 0))),
        safe_observation_candidates=int(s.get("safe_observation_candidates", 0)),
        budget_rejected_candidates=int(s.get("budget_rejected_candidates", 0)),
        rejected_goal_clearance=int(s.get("rejected_goal_clearance", 0)),
        rejected_route_reachability=int(s.get("rejected_route_reachability", 0)),
        rejected_visibility=int(s.get("rejected_visibility", 0)),
        rejected_view_distance=int(s.get("rejected_view_distance", 0)),
        rejected_local_distance=int(s.get("rejected_local_distance", 0)),
        rejected_near_robot=int(s.get("rejected_near_robot", 0)),
        rejected_cooldown=int(s.get("rejected_cooldown", 0)),
        viewpoint_cells_checked=int(s.get("viewpoint_cells_checked", 0)),
        current_target_valid=bool(s.get("current_target_xy")),
        current_target_x=float((s.get("current_target_xy") or [0.0, 0.0])[0]),
        current_target_y=float((s.get("current_target_xy") or [0.0, 0.0])[1]),
    )


@explore_skill.mcp("robonix/skill/explore/explore/cancel")
def cancel(req: CancelExplore_Request) -> CancelExplore_Response:
    """Abort the active exploration. Idempotent."""
    if ctrl is None:
        raise RuntimeError("controller not initialized")
    ok, msg = ctrl.cancel(req.run_id or None)
    return CancelExplore_Response(ok=ok, message=msg)


# ── lifecycle ────────────────────────────────────────────────────────────────
# Skills split init from activate: rbnx boot calls Driver(CMD_INIT) on
# every package and stops there for skills (state = INITIALIZED). The
# executor sends Driver(CMD_ACTIVATE) just-in-time on the first MCP
# call, which is when the skill actually allocates hot resources (ROS
# subs, frontier loop, …). See docs/cap-lifecycle.md for the full FSM.
@explore_skill.on_init
def init(cfg):
    """Validate deployment settings at INIT without connecting to ROS or Atlas."""
    global settings
    try:
        settings = ExploreSettings.parse(cfg)
    except (TypeError, ValueError) as exc:
        return Err(f"invalid explore configuration: {exc}")
    log.info("CMD_INIT ok: timeout_s=%s sweep_every_n_legs=%s",
             settings.timeout_s, settings.sweep_every_n_legs)
    return Ok()


@explore_skill.on_activate
def activate():
    """CMD_ACTIVATE: heavy. Resolve the upstream contracts NOW (executor
    only sends CMD_ACTIVATE when there's actually a request to satisfy,
    by which point map / nav should be ACTIVE), then build the
    ExploreController and start the rclpy thread. Idempotent on re-entry."""
    global ctrl
    if ctrl is not None:
        log.info("CMD_ACTIVATE — already runnable, no-op")
        return Ok()
    inputs = resolve_inputs()
    log.info("dependencies resolved: %s", list(inputs.keys()))
    ctrl = ExploreController(
        map_topic=inputs["map_topic"],
        nav_navigate_endpoint=inputs["nav_navigate"],
        nav_status_endpoint=inputs["nav_status"],
        nav_cancel_endpoint=inputs["nav_cancel"],
        nav_get_speed_endpoint=inputs["nav_get_speed"],
        settings=settings,
        scene_list_objects_endpoint=inputs.get("scene_list_objects"),
    )
    ctrl.start_runtime()
    log.info("CMD_ACTIVATE ok — controller running")
    return Ok()


@explore_skill.on_deactivate
def deactivate():
    """CMD_DEACTIVATE: stop the rclpy thread + drop the controller. Safe
    to call repeatedly; the second call is a no-op. Executor's eviction
    policy fires this when the skill has been idle long enough."""
    global ctrl
    if ctrl is None:
        return Ok()
    try:
        ctrl.stop_runtime()
    finally:
        ctrl = None
    log.info("CMD_DEACTIVATE ok — controller stopped")
    return Ok()


def main() -> int:
    explore_skill.run()
    if ctrl is not None:
        ctrl.stop_runtime()
    return 0


if __name__ == "__main__":
    import sys
    sys.exit(main())
