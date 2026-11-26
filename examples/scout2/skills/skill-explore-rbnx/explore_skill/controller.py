# SPDX-License-Identifier: MulanPSL-2.0
"""Exploration task lifecycle + frontier-loop driver.

State machine per task:

    explore() called ──► EXPLORING ──► (success) ──► DONE
                              │
                              ├──► (deadline)   ──► TIMEOUT
                              ├──► (cancel)     ──► CANCELED
                              └──► (nav errors) ──► ERROR

The controller owns one in-flight task at a time. Concurrent explore()
is rejected with accepted=false. Caller can cancel + retry.

Threading:
  - rclpy thread (ExploreController._spin_thread): hosts the /map
    subscriber and tf2 buffer.
  - task thread (per explore() invocation): runs the frontier loop.
  - heartbeat thread (in atlas_bridge): unrelated.

Nav integration:
  The skill calls service/navigation/navigate and its status/cancel controls as
  gRPC RPCs against the endpoints atlas resolved at startup. We do
  NOT subscribe to /goal_pose or /cmd_vel — that would skip the nav
  service's contract surface and break the abstraction.
"""
from __future__ import annotations

import logging
import math
import threading
import time
import uuid
from dataclasses import dataclass, field
from typing import Any, Optional, Tuple

from .configuration import ExploreSettings

log = logging.getLogger("explore_rbnx.controller")


@dataclass
class TaskHandle:
    task_id: str
    started_at: float
    timeout_s: float                  # 0 = no timeout
    max_speed_m_s: float
    started_monotonic: float = field(default_factory=time.monotonic)
    finished_monotonic: Optional[float] = None
    no_target_since: Optional[float] = None
    state: str = "exploring"          # exploring | done | timeout | canceled | error
    detail: str = "started"
    last_target_xy: Optional[Tuple[float, float]] = None
    frontier_regions: int = 0
    safe_observation_candidates: int = 0
    budget_rejected_candidates: int = 0
    rejected_goal_clearance: int = 0
    rejected_route_reachability: int = 0
    rejected_visibility: int = 0
    rejected_view_distance: int = 0
    rejected_local_distance: int = 0
    rejected_near_robot: int = 0
    rejected_cooldown: int = 0
    viewpoint_cells_checked: int = 0
    current_target_xy: Optional[Tuple[float, float]] = None
    active_nav_run_id: str = ""
    effective_speed_m_s: float = 0.0
    last_sweep_area_m2: Optional[float] = None
    sweep_without_growth: bool = False
    initial_area_m2: float = 0.0
    last_area_m2: float = 0.0
    last_progress_t: float = field(default_factory=time.monotonic)
    last_progress_area_m2: float = 0.0
    last_progress_pose_xy: Optional[Tuple[float, float]] = None
    cancel_requested: bool = False
    legs_completed: int = 0           # successful nav legs in this task
    # Consecutive sweeps tracking (from backup fix for complete mapping)
    consecutive_sweeps: int = 0       # consecutive 360° sweeps in place
    thread: Optional[threading.Thread] = None
    # Failure tracking
    consecutive_nav_fails: int = 0    # consecutive navigation failures
    consecutive_no_target: int = 0    # iterations without a safe frontier


class ExploreController:
    """Frontier-loop controller. One instance per service process;
    one active task at a time."""

    # Tunables
    FRONTIER_MIN_SIZE_CELLS = 3           # below this = noise, ignore
    DONE_QUIET_SECONDS = 30.0             # no progress for this long → done
    PROGRESS_AREA_DELTA_M2 = 0.5          # area gain considered "progress"
    NAV_POLL_PERIOD_S = 1.0
    NAV_CANCEL_TIMEOUT_S = 3.0
    NAV_SELECTION_MARGIN_S = 5.0
    SWEEP_MAX_TIME_FRACTION = 0.15
    LOOP_QUIET_PERIOD_S = 2.0             # delay between consecutive goals
    # "Local exploration" radius: candidates farther than this from
    # current robot pose are skipped. Forces the skill to clear the
    # current room before jumping to a far-away frontier.
    # Increased to 15.0 from 6.0 to allow exploring across larger rooms
    # and corridors without prematurely terminating.
    MAX_FRONTIER_DISTANCE_M = 15.0
    # Mark cells within this radius of the robot as "visited" each
    # time we update the pose. Used to deprioritise re-revisiting
    # already-cleared areas when multiple frontiers tie in score.
    VISITED_RADIUS_M = 0.4
    # Coverage tracking: each cell records which yaw sectors the
    # camera has pointed at. 8 sectors of 45° each. A cell with all
    # 8 sectors filled has been observed from every direction.
    YAW_SECTORS = 8
    # A 360° sweep is expensive for wheel odometry, so only perform it
    # periodically and when the current cell still lacks yaw coverage.
    # Previously the OR condition swept at every newly reached cell,
    # accumulating rotation drift and starving the global timeout.
    SWEEP_MIN_SECTORS = 6
    # Failure thresholds: terminate task when consecutive failures
    # exceed these limits to prevent infinite loops.
    MAX_CONSECUTIVE_NAV_FAILS = 3        # navigation failures in a row
    STATE_STALE_TIMEOUT_S = 120.0        # no confirmed growth/translation → stuck
    PROGRESS_POSE_DELTA_M = 0.25         # confirmed translation, not yaw-only motion
    # Goal memory: remember failed navigation targets to avoid retrying.
    FAILED_GOALS_MEMORY = 10             # max failed goals to remember
    FAILED_GOAL_RADIUS_M = 0.5           # radius around failed goal to penalize
    FAILED_GOAL_COOLDOWN_S = 120.0       # retry a failed area after this delay
    # Dynamic safe radius: adjust based on navigation success rate.
    INITIAL_SAFE_RADIUS_M = 0.65
    MIN_SAFE_RADIUS_M = 0.65
    MAX_SAFE_RADIUS_M = 0.80
    RADIUS_ADJUST_STEP = 0.05
    RADIUS_EVAL_INTERVAL = 5             # evaluate success rate every N legs

    def __init__(self, *, map_topic: str,
                 nav_navigate_endpoint: str,
                 nav_status_endpoint: str,
                 nav_cancel_endpoint: str,
                 scene_list_objects_endpoint: Optional[str] = None,
                 nav_get_speed_endpoint: Optional[str] = None,
                 settings: Optional[ExploreSettings] = None):
        self.map_topic = map_topic
        self.settings = settings or ExploreSettings()
        # All three nav endpoints typically point at the same FastMCP
        # server (http://host:port/mcp/) — atlas hands us the URL each
        # tool registered under. Keep separate fields so a future
        # multi-endpoint nav setup (e.g. nav2-wrapper for some modes,
        # simple_nav for others) still works.
        self._nav_endpoints = {
            "navigate": nav_navigate_endpoint,
            "status":   nav_status_endpoint,
            "cancel":   nav_cancel_endpoint,
            "get_speed_limit": nav_get_speed_endpoint,
        }
        # Scene service endpoint for glass door detection (optional)
        self._scene_endpoint = scene_list_objects_endpoint
        self._lock = threading.Lock()
        self._latest_map: Any = None     # latest OccupancyGrid msg
        self._latest_pose_xyyaw: Optional[Tuple[float, float, float]] = None
        self._latest_pose_received_at: Optional[float] = None
        # Visited cells (set of (cx, cy) cell-space ints). Reset per
        # task so a second explore() doesn't inherit stale state.
        self._visited_cells: set = set()
        # Per-cell viewing-angle coverage: dict[(cx,cy), set[int]]
        # where the int is a sector index in [0, YAW_SECTORS).
        # Used to decide whether to insert a 360° sweep at this cell
        # and to evaluate "fully observed" coverage at done-time.
        self._viewed_sectors: dict = {}
        # Failed goal coordinates and monotonic failure times for cooldown.
        self._failed_goals: dict = {}
        # Glass door positions from scene service: list of (x, y) coords
        self._glass_doors: list = []
        self._last_glass_door_update: float = 0.0
        # Dynamic safe radius for target safety checks.
        self._current_safe_radius: float = self.INITIAL_SAFE_RADIUS_M
        # Navigation success rate tracking for radius adjustment.
        self._nav_success_count: int = 0
        self._nav_total_count: int = 0
        self._task: Optional[TaskHandle] = None

        self._ros: Optional[dict] = None
        self._node = None
        self._spin_thread: Optional[threading.Thread] = None
        self._stop_evt = threading.Event()

        self._tf_buffer = None
        self._tf_listener = None

        # MCP clients are created per RPC/event loop to avoid cross-thread reuse.

    # ── ROS runtime ─────────────────────────────────────────────────
    def start_runtime(self) -> None:
        """Spin up rclpy node + /map subscriber + tf2 listener."""
        if self._ros is not None:
            return
        self._ros = _import_ros()
        rclpy = self._ros["rclpy"]
        rclpy.init(args=None)

        node = self._ros["Node"]("explore_skill")
        self._node = node

        QoS = self._ros["QoSProfile"]
        Rel = self._ros["ReliabilityPolicy"]
        Dur = self._ros["DurabilityPolicy"]
        Hist = self._ros["HistoryPolicy"]
        # /map is published TRANSIENT_LOCAL by mapping; we must match.
        map_qos = QoS(reliability=Rel.RELIABLE,
                      durability=Dur.TRANSIENT_LOCAL,
                      history=Hist.KEEP_LAST, depth=1)
        node.create_subscription(self._ros["OccupancyGrid"],
                                  self.map_topic, self._on_map, map_qos)

        # tf2 for map→base_link lookup (so we know where the robot is)
        from tf2_ros import Buffer, TransformListener
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, node)

        self._stop_evt.clear()
        self._spin_thread = threading.Thread(target=self._spin_loop,
                                              daemon=True)
        self._spin_thread.start()
        log.info("rclpy spinning; subscribed to map_topic=%s", self.map_topic)

    def stop_runtime(self) -> None:
        if self._task and self._task.state == "exploring":
            self.cancel(None)
        self._stop_evt.set()
        if self._spin_thread:
            self._spin_thread.join(timeout=3.0)
        if self._ros:
            try:
                self._ros["rclpy"].shutdown()
            except Exception:
                pass

    def _spin_loop(self) -> None:
        rclpy = self._ros["rclpy"]
        while not self._stop_evt.is_set():
            try:
                rclpy.spin_once(self._node, timeout_sec=0.2)
                self._refresh_robot_pose()
            except Exception as e:  # noqa: BLE001
                log.exception("spin error: %s", e)
                time.sleep(0.1)

    def _on_map(self, msg) -> None:
        """Record real cumulative map growth, including while a nav call blocks."""
        from .frontier import GridView, mapped_free_area_m2
        area_m2 = mapped_free_area_m2(GridView.from_msg(msg))
        with self._lock:
            self._latest_map = msg
            if self._task is not None and self._task.state == "exploring":
                self._record_progress(self._task, area_m2, None)

    def _refresh_robot_pose(self) -> None:
        """Look up map→base_link; record (x,y,yaw); update visited
        cells + per-cell viewing-angle coverage."""
        if self._tf_buffer is None:
            return
        try:
            from rclpy.time import Time
            tr = self._tf_buffer.lookup_transform(
                "map", "base_link", Time(),
                timeout=self._ros["Duration"](seconds=0.5))
        except Exception:
            with self._lock:
                self._latest_pose_received_at = None
            return
        stamp = tr.header.stamp.sec * 1_000_000_000 + tr.header.stamp.nanosec
        age_ns = self._node.get_clock().now().nanoseconds - stamp
        if not -100_000_000 <= age_ns <= 500_000_000:
            with self._lock:
                self._latest_pose_received_at = None
            return
        x = float(tr.transform.translation.x)
        y = float(tr.transform.translation.y)
        # Quaternion → yaw (z rotation only — Force3DoF world).
        q = tr.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        with self._lock:
            self._latest_pose_xyyaw = (x, y, yaw)
            self._latest_pose_received_at = time.monotonic()
            self._mark_visited(x, y, yaw)
            if self._task is not None and self._task.state == "exploring":
                self._record_progress(self._task, None, (x, y))

    def _mark_visited(self, x: float, y: float, yaw: float) -> None:
        """Stamp visited cells (radius-disk) and add the current yaw
        sector to the per-cell coverage map. Called with self._lock."""
        if self._latest_map is None:
            return
        from .frontier import GridView
        gv = GridView.from_msg(self._latest_map)
        cx, cy = gv.world_to_cell(x, y)
        r = max(1, int(round(self.VISITED_RADIUS_M / gv.resolution)))
        for dy in range(-r, r + 1):
            for dx in range(-r, r + 1):
                if dx * dx + dy * dy <= r * r:
                    self._visited_cells.add((cx + dx, cy + dy))
        # Yaw normalised to [0, 2π) → sector index in [0, YAW_SECTORS).
        sector = int(((yaw % (2 * math.pi)) / (2 * math.pi))
                     * self.YAW_SECTORS) % self.YAW_SECTORS
        self._viewed_sectors.setdefault((cx, cy), set()).add(sector)

    # ── Public API (called from MCP tool handlers) ─────────────────
    def start(self, *, area_hint: str, timeout_s: float,
              max_speed_m_s: float) -> TaskHandle:
        """Validate limits and start one owned task, reporting the effective timeout."""
        if not math.isfinite(timeout_s) or timeout_s < 0:
            raise ValueError("resolved timeout_s must be finite and nonnegative")
        if not math.isfinite(max_speed_m_s) or max_speed_m_s <= 0:
            raise ValueError("max_speed_m_s must be finite and positive")
        with self._lock:
            if self._task is not None and (self._task.state == "exploring"
                    or self._task.active_nav_run_id
                    or (self._task.thread is not None and self._task.thread.is_alive())):
                raise RuntimeError(
                    f"another exploration task is already running "
                    f"(task_id={self._task.task_id}). Call cancel() "
                    f"before starting a new one.")
            # Reset visited history for the new task so we don't carry
            # over stale cells from a prior incomplete run.
            self._visited_cells = set()
            self._failed_goals = {}  # reset failed goals
            self._viewed_sectors = {}
            self._current_safe_radius = self.INITIAL_SAFE_RADIUS_M
            self._nav_total_count = 0
            self._nav_success_count = 0
            handle = TaskHandle(
                task_id="exp-" + uuid.uuid4().hex[:8],
                started_at=time.time(),
                timeout_s=float(timeout_s),
                max_speed_m_s=float(max_speed_m_s),
                detail=f"started (area_hint={area_hint!r}, timeout_s={timeout_s:g})",
            )
            self._task = handle
        handle.thread = threading.Thread(
            target=self._run_task, args=(handle,),
            name=f"explore-{handle.task_id}", daemon=True)
        handle.thread.start()
        log.info("[%s] effective timeout_s=%s max_speed_m_s=%s",
                 handle.task_id, handle.timeout_s, handle.max_speed_m_s)
        return handle

    def cancel(self, task_id: Optional[str]) -> Tuple[bool, str]:
        """Cancel this task's action, including retrying unconfirmed terminal cleanup."""
        with self._lock:
            task = self._task
            if task is None:
                return True, "no active task (no-op)"
            if task_id and task.task_id != task_id:
                return False, f"task_id mismatch (active={task.task_id})"
            terminal = task.state != "exploring"
            run_id = task.active_nav_run_id
            if terminal and not run_id:
                return True, f"task already in terminal state {task.state} (no-op)"
            task.cancel_requested = True
            task.detail = "cancel requested"
        if terminal:
            if self._cancel_navigation(run_id):
                with self._lock:
                    task.active_nav_run_id = ""
                return True, "navigation cancellation confirmed"
            return False, "navigation cancellation was not confirmed"
        if run_id:
            self._nav_cancel_rpc(run_id=run_id)
        return True, f"cancel requested for {task.task_id}"

    def status(self, task_id: Optional[str]) -> Optional[dict]:
        with self._lock:
            t = self._task
            if t is None:
                return None
            if task_id and t.task_id != task_id:
                return None
            ended = t.finished_monotonic if t.finished_monotonic is not None else time.monotonic()
            elapsed = max(0.0, ended - t.started_monotonic)
            return {
                "state": t.state,
                "area_m2": float(t.last_area_m2),
                "frontiers_left": _frontiers_left_count(self._latest_map),
                "elapsed_s": float(elapsed),
                "eta_s": -1.0,    # we don't currently estimate
                "detail": t.detail,
                "frontier_regions": int(t.frontier_regions),
                "safe_observation_candidates": int(t.safe_observation_candidates),
                "budget_rejected_candidates": int(t.budget_rejected_candidates),
                "rejected_goal_clearance": int(t.rejected_goal_clearance),
                "rejected_route_reachability": int(t.rejected_route_reachability),
                "rejected_visibility": int(t.rejected_visibility),
                "rejected_view_distance": int(t.rejected_view_distance),
                "rejected_local_distance": int(t.rejected_local_distance),
                "rejected_near_robot": int(t.rejected_near_robot),
                "rejected_cooldown": int(t.rejected_cooldown),
                "viewpoint_cells_checked": int(t.viewpoint_cells_checked),
                "current_target_xy": list(t.current_target_xy) if t.current_target_xy else [],
                "task_id": t.task_id,
            }

    # ── Frontier loop (runs in its own thread per task) ─────────────
    def _run_task(self, handle: TaskHandle) -> None:
        """Contain RPC/data failures and cancel the exact navigation run on exit."""
        try:
            self._run_task_loop(handle)
        except Exception as exc:  # noqa: BLE001
            log.exception("[%s] exploration failed", handle.task_id)
            self._terminate(handle, "error", f"exploration failed: {exc}")
        finally:
            if handle.active_nav_run_id:
                if not self._cancel_navigation(handle.active_nav_run_id):
                    self._terminate(handle, "error", "navigation cancellation was not confirmed")
                else:
                    handle.active_nav_run_id = ""

    def _run_task_loop(self, handle: TaskHandle) -> None:
        """Select feasible frontier legs within both per-leg and remaining budgets."""
        from .frontier import (GridView, mapped_free_area_m2, pick_target,
                                total_frontier_count)
        log.info("[%s] exploration task starting", handle.task_id)
        deadline = handle.started_monotonic + handle.timeout_s if handle.timeout_s > 0 else None
        # Startup waiting also consumes the requested total budget.
        gm = self._wait_for_map(timeout_s=self._bounded_timeout(15.0, deadline))
        if gm is None:
            timed_out = deadline is not None and time.monotonic() >= deadline
            self._terminate(handle, "timeout" if timed_out else "error",
                             "no map before task deadline" if timed_out else "no map received within 15s")
            return
        gv = GridView.from_msg(gm)
        with self._lock:
            handle.initial_area_m2 = mapped_free_area_m2(gv)
            handle.last_area_m2 = handle.initial_area_m2
            handle.last_progress_area_m2 = handle.initial_area_m2
            pose = self._latest_pose_xyyaw
            handle.last_progress_pose_xy = (pose[0], pose[1]) if pose else None

        while True:
            # Cancel / timeout checks.
            if handle.cancel_requested:
                self._terminate(handle, "canceled", "user-requested cancel")
                return
            if deadline is not None and time.monotonic() > deadline:
                self._terminate(handle, "timeout",
                                 f"hit {handle.timeout_s}s ceiling")
                return

            # Update glass door detections from scene service
            self._update_glass_doors()

            # Get latest map + pose.
            with self._lock:
                latest = self._latest_map
                pose_xyyaw = self._latest_pose_xyyaw
            if latest is None:
                time.sleep(0.5)
                continue
            pose = (pose_xyyaw[0], pose_xyyaw[1]) if pose_xyyaw else None
            gv = GridView.from_msg(latest)
            cur_area = mapped_free_area_m2(gv)
            handle.last_area_m2 = cur_area
            with self._lock:
                self._record_progress(handle, cur_area, pose)
            if self._progress_stale(handle):
                self._terminate(handle, "error",
                                f"no confirmed map growth or translation for "
                                f"{self.STATE_STALE_TIMEOUT_S:.0f}s")
                return
            n_frontiers = total_frontier_count(
                gv, min_size=self.FRONTIER_MIN_SIZE_CELLS)

            # Pick next target.
            if pose is None:
                log.info("[%s] no map→base_link tf yet, waiting...",
                          handle.task_id)
                time.sleep(0.5)
                continue
            with self._lock:
                visited_snapshot = set(self._visited_cells)
                failed_snapshot = dict(self._failed_goals)
                safe_radius = self._current_safe_radius
                glass_doors_snapshot = list(self._glass_doors)
            speed = self._navigation_speed(handle)
            time_budget = self._leg_budget(deadline)
            reference_budget = self._reference_leg_budget(time_budget)
            max_route = self._max_route_distance(time_budget, speed)
            preferred_route = self._max_route_distance(reference_budget, speed)
            target_options = dict(
                min_size=self.FRONTIER_MIN_SIZE_CELLS,
                max_distance_m=self.MAX_FRONTIER_DISTANCE_M,
                visited_cells=visited_snapshot,
                failed_goals=failed_snapshot,
                failed_goal_radius_m=self.FAILED_GOAL_RADIUS_M,
                failed_goal_cooldown_s=self.FAILED_GOAL_COOLDOWN_S,
                safe_radius_m=safe_radius,
                arrival_tolerance_m=self.settings.arrival_tolerance_m,
                glass_doors=glass_doors_snapshot,
                preferred_travel_distance_m=preferred_route,
                allow_route_splitting=(time_budget >= self.settings.nav_goal_min_timeout_s),
            )
            selection = {}
            target = pick_target(gv, pose, max_travel_distance_m=max_route,
                                 diagnostics=selection, **target_options)
            handle.frontier_regions = n_frontiers
            handle.safe_observation_candidates = int(selection.get('ranked_goals', 0))
            handle.budget_rejected_candidates = int(selection.get('rejected_budget', 0))
            handle.rejected_goal_clearance = int(selection.get('rejected_goal_clearance', 0))
            handle.rejected_route_reachability = int(selection.get('rejected_route_reachability', 0))
            handle.rejected_visibility = int(selection.get('rejected_visibility', 0))
            handle.rejected_view_distance = int(selection.get('rejected_view_distance', 0))
            handle.rejected_local_distance = int(selection.get('rejected_distance', 0))
            handle.rejected_near_robot = int(selection.get('rejected_near_robot', 0))
            handle.rejected_cooldown = int(selection.get('rejected_cooldown', 0))
            handle.viewpoint_cells_checked = int(selection.get('viewpoint_cells_checked', 0))
            handle.current_target_xy = None if target is None else target.centroid_xy
            if target is None and selection.get('ranked_goals', 0) > 0:
                remaining_limited = deadline is not None and deadline - time.monotonic() < self.settings.nav_goal_timeout_s
                split_detail = ("no safe intermediate observation waypoint fits"
                                if selection.get('route_split_unavailable')
                                else "no observation route fits")
                self._terminate(handle, "timeout" if remaining_limited else "error",
                    f"safe frontiers remain, but {split_detail}; advisory "
                    f"reference={reference_budget:.1f}s, hard leg limit="
                    f"{time_budget:.1f}s at {speed:.3f}m/s; "
                    f"shortest full route estimate "
                    f"{self._estimate_leg_seconds(selection['min_route_distance_m'], speed):.1f}s; "
                    f"{self._selection_detail(selection)}")
                return

            # Completion needs zero frontiers. Filtered/unreachable frontiers
            # are reported as a limitation rather than successful coverage.
            no_progress = time.monotonic() - handle.last_progress_t > self.DONE_QUIET_SECONDS
            if no_progress and n_frontiers == 0:
                self._terminate(handle, "done",
                                "no frontiers + no progress for "
                                f"{self.DONE_QUIET_SECONDS:.0f}s "
                                f"(area={cur_area:.1f}m²)")
                return
            if n_frontiers == 0:
                # Wait through the quiet window without inventing new sweeps
                # or consuming failure retries on an already closed boundary.
                handle.detail = "no frontiers; waiting for map to settle"
                time.sleep(self.LOOP_QUIET_PERIOD_S)
                continue

            if target is None:
                if selection.get('robot_clearance_safe') is False:
                    arrival_failure = (f"{handle.detail}; "
                                       if handle.detail.startswith("unsafe arrival:") else "")
                    self._terminate(handle, "error",
                        f"{arrival_failure}robot start pose is outside the known-free 0.65m footprint-clear map; "
                        f"{self._selection_detail(selection)}. No frontier goal was sent; "
                        f"map reset is not implied")
                    return
                now = time.monotonic()
                if handle.no_target_since is None:
                    handle.no_target_since = now
                # New map growth or real translation may restore candidates;
                # five fast polls of one static map do not constitute failure.
                quiet_since = max(handle.no_target_since, handle.last_progress_t)
                quiet_s = max(0.0, now - quiet_since)
                diagnostics = self._selection_detail(selection)
                if quiet_s >= self.DONE_QUIET_SECONDS:
                    self._terminate(handle, "error",
                        f"partial mapping; exploration blocked: {n_frontiers} frontiers remain "
                        f"but no safe observable reachable goal was found after "
                        f"{quiet_s:.1f}s without new progress; map is not reported complete; "
                        f"{diagnostics}")
                    return
                handle.consecutive_no_target += 1
                handle.consecutive_nav_fails = 0
                policy = "disabled" if self.settings.sweep_every_n_legs == 0 else "budget/coverage gated"
                handle.detail = (f"frontier_regions={n_frontiers}, "
                    f"safe_candidates={selection.get('ranked_goals', 0)}, "
                    f"budget_rejected={selection.get('rejected_budget', 0)}; "
                    f"no safe observation target; waiting for map/TF update "
                    f"({quiet_s:.1f}/{self.DONE_QUIET_SECONDS:g}s, "
                    f"sweeps {policy}); {diagnostics}")
                log.info("[%s] %s", handle.task_id, handle.detail)
                if self.settings.sweep_every_n_legs > 0 and not handle.sweep_without_growth:
                    self._sweep_360(handle, deadline=deadline)
                    if handle.state != "exploring":
                        return
                # This wait is required even if a sweep is skipped by policy.
                time.sleep(self._bounded_timeout(self.LOOP_QUIET_PERIOD_S, deadline))
                continue
            handle.no_target_since = None

            tx, ty = target.centroid_xy
            estimate_s = self._estimate_leg_seconds(target.travel_distance_m, speed)
            # Scoring/RPC latency may consume global time. Re-select once with
            # the updated allowance instead of sending a now-infeasible route.
            hard_leg_limit = self._leg_budget(deadline)
            if estimate_s > hard_leg_limit + 1e-9:
                if hard_leg_limit < self.settings.nav_goal_min_timeout_s:
                    self._terminate(handle, "timeout", "remaining task time cannot fit a safe navigation leg")
                    return
                handle.detail = (f"map or RPC latency reduced the hard leg limit below "
                    f"the {estimate_s:.1f}s route estimate; reselecting a safe target")
                time.sleep(self._bounded_timeout(self.LOOP_QUIET_PERIOD_S, deadline))
                continue
            handle.last_target_xy = (tx, ty)
            handle.current_target_xy = (tx, ty)
            frontier_goal = getattr(target, 'frontier_goal_xy', None)
            waypoint_note = (f"; intermediate waypoint toward frontier observation target "
                f"({frontier_goal[0]:.2f},{frontier_goal[1]:.2f})"
                if frontier_goal else "")
            reference_note = ("; reference exceeded; trying within hard limit"
                              if estimate_s > reference_budget else "")
            handle.detail = (f"frontier_regions={n_frontiers}, "
                f"safe_candidates={handle.safe_observation_candidates}, "
                f"budget_rejected={handle.budget_rejected_candidates}; "
                f"driving to {'safe waypoint' if frontier_goal else 'frontier observation target'} "
                f"({tx:.2f},{ty:.2f}) segment_size={target.size}"
                f"{waypoint_note}; route_estimate={estimate_s:.1f}s, "
                f"reference={reference_budget:.1f}s, hard_limit={hard_leg_limit:.1f}s "
                f"(grid estimate, actual travel time uncertain){reference_note}")
            log.info("[%s] %s", handle.task_id, handle.detail)
            log.info("[%s] route=%.2fm speed=%.3fm/s leg_limit=%.1fs remaining=%.1fs",
                handle.task_id, target.travel_distance_m, speed, hard_leg_limit,
                self._bounded_timeout(float('inf'), deadline))
            ok, msg = self._nav_navigate_blocking(
                tx, ty, yaw=None, timeout_s=hard_leg_limit,
                cancel_evt=handle)
            if handle.state != "exploring":
                return
            if handle.cancel_requested:
                self._terminate(handle, "canceled", "cancel during nav")
                return
            if deadline is not None and time.monotonic() >= deadline:
                self._terminate(handle, "timeout", f"hit {handle.timeout_s:g}s ceiling")
                return
            if not ok:
                if msg.startswith("unsafe arrival:"):
                    with self._lock:
                        if handle.last_target_xy:
                            self._failed_goals[handle.last_target_xy] = time.monotonic()
                            if len(self._failed_goals) > self.FAILED_GOALS_MEMORY:
                                oldest = min(self._failed_goals, key=self._failed_goals.get)
                                del self._failed_goals[oldest]
                    handle.detail = msg
                    handle.consecutive_nav_fails = 0
                    log.warning("[%s] %s; selecting another observation target",
                                handle.task_id, msg)
                    time.sleep(self.LOOP_QUIET_PERIOD_S)
                    continue
                # Nav failed for this leg — that's not fatal, the next
                # frontier might be reachable. Just log + continue.
                handle.consecutive_nav_fails += 1
                handle.consecutive_no_target = 0  # reset other counter

                # Record failed goal to avoid retrying same target
                if handle.last_target_xy:
                    with self._lock:
                        self._failed_goals[handle.last_target_xy] = time.monotonic()
                        if len(self._failed_goals) > self.FAILED_GOALS_MEMORY:
                            oldest = min(self._failed_goals, key=self._failed_goals.get)
                            del self._failed_goals[oldest]

                # Update success rate tracking
                self._nav_total_count += 1
                self._adjust_safe_radius()

                # Check if exceeded threshold
                if handle.consecutive_nav_fails >= self.MAX_CONSECUTIVE_NAV_FAILS:
                    self._terminate(handle, "error",
                                    f"too many consecutive nav fails ({handle.consecutive_nav_fails})")
                    return

                log.warning("[%s] nav leg failed (%d/%d): %s",
                            handle.task_id,
                            handle.consecutive_nav_fails,
                            self.MAX_CONSECUTIVE_NAV_FAILS,
                            msg)
                handle.detail = f"nav leg failed ({handle.consecutive_nav_fails}/{self.MAX_CONSECUTIVE_NAV_FAILS}); trying next frontier"
                time.sleep(self.LOOP_QUIET_PERIOD_S)
                continue

            handle.legs_completed += 1
            handle.consecutive_nav_fails = 0    # reset nav failure counter
            handle.consecutive_no_target = 0    # reset no-target counter
            handle.consecutive_sweeps = 0       # reset sweep counter after successful nav
            with self._lock:
                pose_xyyaw = self._latest_pose_xyyaw
                latest = self._latest_map
            if pose_xyyaw is not None and latest is not None:
                from .frontier import GridView, mapped_free_area_m2
                area = mapped_free_area_m2(GridView.from_msg(latest))
                with self._lock:
                    self._record_progress(handle, area, (pose_xyyaw[0], pose_xyyaw[1]))

            # Update success rate tracking
            self._nav_success_count += 1
            self._nav_total_count += 1
            self._adjust_safe_radius()

            # 360° sweep to fill camera viewing-angle coverage at the
            # leg endpoint. Two triggers:
            #   - the cell's covered yaw-sectors < SWEEP_MIN_SECTORS,
            #     i.e. we haven't looked around enough here yet
            #   - or every Nth leg as a periodic safety, in case the
            #     coverage tracker missed something
            if self._should_sweep(handle):
                self._sweep_360(handle, deadline=deadline)
                if handle.state != "exploring":
                    return

            # Brief settle before re-evaluating frontiers.
            time.sleep(self._bounded_timeout(self.LOOP_QUIET_PERIOD_S, deadline))

    def _terminate(self, handle: TaskHandle, state: str, detail: str) -> None:
        with self._lock:
            handle.state = state
            handle.detail = detail
            if handle.finished_monotonic is None:
                handle.finished_monotonic = time.monotonic()
        log.info("[%s] task %s: %s", handle.task_id, state, detail)

    @staticmethod
    def _selection_detail(selection: dict) -> str:
        """Expose why existing boundaries did not become navigation goals."""
        return (f"robot_clearance_safe={selection.get('robot_clearance_safe', False)}, "
                f"reachable_cells={selection.get('reachable_cells', 0)}, "
                f"viewpoints={selection.get('generated_viewpoints', 0)}, "
                f"goal_clearance_radius_m={selection.get('goal_clearance_radius_m', 0):.2f}, "
                f"goal_clearance_candidates={selection.get('goal_clearance_candidates', 0)}, "
                f"rejected_far={selection.get('rejected_distance', 0)}, "
                f"rejected_near={selection.get('rejected_near_robot', 0)}, "
                f"rejected_cooldown={selection.get('rejected_cooldown', 0)}, "
                f"rejected_budget={selection.get('rejected_budget', 0)}, "
                f"rejected_goal_clearance={selection.get('rejected_goal_clearance', 0)}, "
                f"rejected_route_reachability={selection.get('rejected_route_reachability', 0)}, "
                f"rejected_visibility={selection.get('rejected_visibility', 0)}, "
                f"rejected_view_distance={selection.get('rejected_view_distance', 0)}, "
                f"rejected_local_distance={selection.get('rejected_distance', 0)}, "
                f"viewpoint_cells_checked={selection.get('viewpoint_cells_checked', 0)}, "
                f"feasible_goals={selection.get('feasible_goals', 0)}, "
                f"intermediate_waypoint={selection.get('intermediate_waypoint', False)}, "
                f"route_split_unavailable={selection.get('route_split_unavailable', False)}")

    def _record_progress(self, handle: TaskHandle, area_m2: Optional[float],
                         pose_xy: Optional[Tuple[float, float]]) -> bool:
        """Refresh the no-progress clock only after area gain or translation.

        Area is compared with the last confirmed-progress snapshot, not the
        preceding loop sample, so gradual growth accumulates. Position is
        measured from an independent translation anchor, so slow growth and
        slow translation can each accumulate. Called with self._lock held.
        In-place sweeps and RPC success alone do not extend the clock.
        """
        area_progress = (area_m2 is not None and math.isfinite(area_m2)
                         and area_m2 - handle.last_progress_area_m2 >= self.PROGRESS_AREA_DELTA_M2)
        pose_valid = pose_xy is not None and all(math.isfinite(v) for v in pose_xy)
        pose_progress = (pose_valid and handle.last_progress_pose_xy is not None
                         and math.dist(pose_xy, handle.last_progress_pose_xy) >= self.PROGRESS_POSE_DELTA_M)
        progressed = area_progress or pose_progress
        if progressed:
            handle.last_progress_t = time.monotonic()
        if area_progress:
            handle.last_progress_area_m2 = area_m2
        if pose_progress or (handle.last_progress_pose_xy is None and pose_valid):
            handle.last_progress_pose_xy = pose_xy
        return progressed

    def _progress_stale(self, handle: TaskHandle) -> bool:
        """Report stalls from the monotonic time of the last measured progress."""
        return time.monotonic() - handle.last_progress_t > self.STATE_STALE_TIMEOUT_S

    def _wait_for_map(self, timeout_s: float) -> Any:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            with self._lock:
                if self._latest_map is not None:
                    return self._latest_map
            time.sleep(0.2)
        return None

    # ── Sweep / coverage helpers ───────────────────────────────────
    def _should_sweep(self, handle: TaskHandle) -> bool:
        """Only opt-in periodic sweeps with missing coverage and measurable benefit."""
        every = self.settings.sweep_every_n_legs
        if every <= 0 or handle.legs_completed <= 0 or handle.legs_completed % every != 0:
            return False
        with self._lock:
            pose, latest = self._latest_pose_xyyaw, self._latest_map
        if pose is None or latest is None or handle.sweep_without_growth:
            return False
        from .frontier import GridView
        gv = GridView.from_msg(latest)
        cell = gv.world_to_cell(pose[0], pose[1])
        if handle.last_sweep_area_m2 is not None and handle.last_area_m2 - handle.last_sweep_area_m2 < self.PROGRESS_AREA_DELTA_M2:
            return False
        return len(self._viewed_sectors.get(cell, set())) < self.SWEEP_MIN_SECTORS

    def _navigation_speed(self, handle: TaskHandle) -> float:
        """Read the live navigation limit; never assume the request is actual speed."""
        if not self._nav_endpoints.get("get_speed_limit"):
            raise RuntimeError("navigation get_speed_limit is required for travel budgeting")
        response = self._mcp_call_sync("get_speed_limit", {})
        speed = float(response.get("effective_linear_speed_mps", 0.0))
        if not response.get("available") or not math.isfinite(speed) or speed <= 0:
            raise RuntimeError("navigation speed limit is unavailable")
        if speed > handle.max_speed_m_s + 1e-6:
            raise RuntimeError(f"navigation speed limit {speed:.3f}m/s exceeds requested exploration cap "
                               f"{handle.max_speed_m_s:.3f}m/s; lower navigation speed_limit first")
        handle.effective_speed_m_s = speed
        return handle.effective_speed_m_s

    def _leg_budget(self, deadline: Optional[float]) -> float:
        """Bound actual execution; reserve cleanup only against the task deadline.

        Unlimited exploration uses the full per-leg cap. Finite exploration
        reserves cancellation and settling once against remaining total time,
        without subtracting optional inter-leg sleeps from every action.
        """
        if deadline is None:
            return self.settings.nav_goal_timeout_s
        remaining = max(0.0, deadline - time.monotonic())
        reserve = self.NAV_CANCEL_TIMEOUT_S + self.settings.arrival_settle_s
        return min(self.settings.nav_goal_timeout_s, max(0.0, remaining - reserve))

    def _reference_leg_budget(self, hard_budget_s: float) -> float:
        """Keep the previous overhead-adjusted budget as advisory diagnostics."""
        reference = max(0.0, self.settings.nav_goal_timeout_s - self.NAV_CANCEL_TIMEOUT_S
                        - self.LOOP_QUIET_PERIOD_S - self.NAV_SELECTION_MARGIN_S)
        return min(hard_budget_s, reference)

    def _estimate_leg_seconds(self, route_m: float, speed: float) -> float:
        """Allow for known-space route length, lower actual speed, turns and settling."""
        return max(self.settings.nav_goal_min_timeout_s,
                   self.settings.nav_turn_allowance_s
                   + self.settings.nav_time_factor * route_m / speed
                   + self.settings.arrival_settle_s + self.NAV_POLL_PERIOD_S)

    def _max_route_distance(self, budget_s: float, speed: float) -> float:
        """Filter destinations that cannot fit the current time allowance."""
        fixed = (self.settings.nav_turn_allowance_s + self.settings.arrival_settle_s
                 + self.NAV_POLL_PERIOD_S)
        if budget_s < max(self.settings.nav_goal_min_timeout_s, fixed):
            return 0.0
        return max(0.0, (budget_s - fixed) * speed
                   / self.settings.nav_time_factor)

    @staticmethod
    def _bounded_timeout(requested_s: float,
                         deadline: Optional[float]) -> float:
        if deadline is None:
            return requested_s
        return max(0.0, min(requested_s, deadline - time.monotonic()))

    def _sweep_360(self, handle: TaskHandle, *,
                   deadline: Optional[float]) -> None:
        """Send 4 nav goals at the current xy with yaw advanced 90°
        each time. Robot rotates in place 4 times → 360° sweep. The
        camera covers all 8 sectors over those 4 stops. Per-sector
        update is automatic via _refresh_robot_pose() ticks during
        the rotation."""
        with self._lock:
            pose = self._latest_pose_xyyaw
        if pose is None or self.settings.sweep_every_n_legs == 0 or handle.sweep_without_growth:
            return
        # Budget a complete sweep, rather than starting four rotations with
        # insufficient time. No hard deadline still has a per-sweep bound.
        sweep_budget = 4 * (15.5 + self.NAV_CANCEL_TIMEOUT_S)
        if deadline is not None:
            remaining = deadline - time.monotonic()
            if remaining < sweep_budget + self.NAV_CANCEL_TIMEOUT_S:
                return
            if handle.timeout_s > 0 and sweep_budget > handle.timeout_s * self.SWEEP_MAX_TIME_FRACTION:
                return
        from .frontier import GridView, mapped_free_area_m2
        with self._lock:
            latest = self._latest_map
        if latest is None:
            return
        before_area = mapped_free_area_m2(GridView.from_msg(latest))
        x, y, yaw = pose
        for step in (0.5 * math.pi, math.pi, 1.5 * math.pi, 2.0 * math.pi):
            if handle.cancel_requested:
                return
            segment_timeout = self._bounded_timeout(15.0, deadline)
            if segment_timeout <= 0.0:
                return
            target_yaw = yaw + step
            handle.detail = (f"sweep at ({x:.2f},{y:.2f}) "
                             f"yaw={math.degrees(target_yaw)%360:.0f}°")
            log.info("[%s] %s", handle.task_id, handle.detail)
            ok, _ = self._nav_navigate_blocking(
                x, y, yaw=target_yaw, timeout_s=segment_timeout,
                cancel_evt=handle)
            if handle.state != "exploring":
                return
            if not ok:
                log.warning("[%s] sweep step yaw=%s failed; stopping this sweep",
                            handle.task_id, math.degrees(target_yaw) % 360)
                handle.sweep_without_growth = True
                return
            time.sleep(0.5)
        with self._lock:
            latest = self._latest_map
        if latest is not None:
            from .frontier import GridView, mapped_free_area_m2
            after_area = mapped_free_area_m2(GridView.from_msg(latest))
            handle.last_sweep_area_m2 = after_area
            handle.sweep_without_growth = after_area - before_area < self.PROGRESS_AREA_DELTA_M2
            if handle.sweep_without_growth:
                log.info("[%s] sweep added no meaningful free area; further sweeps disabled",
                         handle.task_id)

    # ── Nav RPC over MCP HTTP ──────────────────────────────────────
    # simple_nav exposes navigate/status/cancel as MCP tools, so we
    # call them via FastMCP's client. atlas_bridge resolved the MCP
    # endpoint URL for each contract; in practice all three point at
    # the same FastMCP server, but we keep them separate so a future
    # multi-nav setup still works.
    def _ensure_mcp_client(self, endpoint: Optional[str] = None):
        """Create a client for one RPC and event loop, without cross-thread reuse."""
        from fastmcp import Client
        return Client(endpoint or self._nav_endpoints["navigate"])

    async def _mcp_call(self, tool: str, args: dict,
                        endpoint: Optional[str] = None) -> dict:
        """Single MCP tool round-trip. Async because fastmcp's client
        is async — we await inside a fresh event loop in the caller.
        If endpoint is provided, use it instead of default nav endpoint."""
        client = self._ensure_mcp_client(endpoint or self._nav_endpoints.get(tool))
        async with client as c:
            result = await c.call_tool(tool, args)
            # FastMCP returns a list of TextContent; the tool returned
            # JSON-serialised dict in its sole entry.
            if not result.content:
                return {}
            import json
            txt = result.content[0].text
            try:
                return json.loads(txt)
            except Exception:
                return {"raw": txt}

    def _mcp_call_sync(self, tool: str, args: dict,
                       endpoint: Optional[str] = None, *, timeout_s: float = 5.0) -> dict:
        import asyncio
        try:
            return asyncio.run(asyncio.wait_for(self._mcp_call(tool, args, endpoint), timeout=max(0.001, timeout_s)))
        except Exception as e:  # noqa: BLE001
            log.warning("mcp call %s failed: %s", tool, e)
            return {}

    def _nav_navigate_blocking(self, x: float, y: float, *,
                                 yaw: Optional[float],
                                 timeout_s: float,
                                 cancel_evt: TaskHandle) -> Tuple[bool, str]:
        """Navigate with measured XY arrival and bounded cancellation.

        Translation legs face their direction of travel, not map yaw zero.
        Stable arrival inside the exploration observation radius ends the leg
        without waiting for a precise final heading. Sweeps require Nav2's yaw
        success and never use this XY-only completion rule.
        """
        if cancel_evt.cancel_requested:
            return False, "canceled before navigation"
        translation = yaw is None
        with self._lock:
            pose = self._latest_pose_xyyaw
        if translation:
            if pose is None:
                return False, "no robot pose for navigation heading"
            yaw = math.atan2(y - pose[1], x - pose[0])
        goal_pose = {
            "header": {"frame_id": "map", "stamp": {"sec": 0, "nanosec": 0}},
            "pose": {"position": {"x": float(x), "y": float(y), "z": 0.0},
                     "orientation": {"x": 0.0, "y": 0.0,
                                     "z": math.sin(yaw / 2), "w": math.cos(yaw / 2)}},
        }
        deadline = time.monotonic() + timeout_s
        resp = self._mcp_call_sync("navigate", {"goal": goal_pose},
                                   timeout_s=min(5.0, max(0.001, timeout_s)))
        if not resp.get("accepted", False):
            return False, f"goal rejected: {resp.get('detail', '')}"
        run_id = resp.get("run_id", "")
        if not run_id:
            raise RuntimeError("navigation accepted goal without run_id")
        cancel_evt.active_nav_run_id = run_id
        inside_since = None
        translation_anchor = pose[:2] if pose is not None else None
        progress_at = time.monotonic() + self.settings.nav_turn_allowance_s
        reason = "leg timeout"
        arrived = False
        while time.monotonic() < deadline:
            if cancel_evt.cancel_requested:
                reason = "canceled during nav"
                break
            response = self._mcp_call_sync("status", {"run_id": run_id},
                timeout_s=min(5.0, max(0.001, deadline - time.monotonic())))
            state = str(response.get("state", "")).upper()
            if response.get("known", True) and state in ("SUCCEEDED", "FAILED", "CANCELED", "TIMEOUT"):
                cancel_evt.active_nav_run_id = ""
                detail = f"nav terminal: {state}; {response.get('detail', '')}"
                if state == "SUCCEEDED" and translation:
                    safe, arrival_detail = self._wait_for_safe_arrival(
                        x, y, deadline, cancel_evt=cancel_evt)
                    if not safe:
                        return False, arrival_detail
                return state == "SUCCEEDED", detail
            now = time.monotonic()
            with self._lock:
                pose = self._latest_pose_xyyaw
                received_at = self._latest_pose_received_at
            if translation and pose is not None and received_at is not None and now - received_at <= 1.0:
                distance = math.hypot(x - pose[0], y - pose[1])
                if distance <= self.settings.arrival_tolerance_m:
                    inside_since = now if inside_since is None else inside_since
                    if now - inside_since >= self.settings.arrival_settle_s:
                        reason = "frontier observation position reached"
                        arrived = True
                        break
                else:
                    inside_since = None
                if translation_anchor is None or math.dist(pose[:2], translation_anchor) >= 0.10:
                    translation_anchor, progress_at = pose[:2], now
                elif now - progress_at >= self.settings.nav_stall_timeout_s:
                    reason = f"navigation made no translation progress for {self.settings.nav_stall_timeout_s:g}s"
                    break
            else:
                inside_since = None
            time.sleep(min(self.NAV_POLL_PERIOD_S, max(0.0, deadline - time.monotonic())))
        if not self._cancel_navigation(run_id):
            self._terminate(cancel_evt, "error", f"{reason}; navigation cancellation was not confirmed")
            return False, cancel_evt.detail
        cancel_evt.active_nav_run_id = ""
        if arrived and translation:
            safe, arrival_detail = self._wait_for_safe_arrival(
                x, y, deadline, cancel_evt=cancel_evt)
            if not safe:
                return False, arrival_detail
            return True, f"{reason}; {arrival_detail}"
        return arrived, reason

    def _wait_for_safe_arrival(self, target_x: float, target_y: float,
                               deadline: float, *,
                               cancel_evt: TaskHandle) -> Tuple[bool, str]:
        """Require a fresh, settled XY pose whose actual chassis disk is known free.

        Nav2 success alone does not authorize exploration progress: the latest
        pose must stay inside the observation radius and pass the original
        footprint-clearance mask for the configured settle interval.
        """
        from .frontier import GridView, is_target_safe
        settled_since = None
        last_pose = None
        while time.monotonic() < deadline:
            if cancel_evt.cancel_requested:
                return False, "canceled during arrival verification"
            now = time.monotonic()
            with self._lock:
                latest = self._latest_map
                pose = self._latest_pose_xyyaw
                received_at = self._latest_pose_received_at
            if (latest is not None and pose is not None and received_at is not None
                    and 0 <= now - received_at <= 1.0):
                grid = GridView.from_msg(latest)
                distance = math.hypot(target_x - pose[0], target_y - pose[1])
                if distance <= self.settings.arrival_tolerance_m:
                    if not is_target_safe(grid, pose[0], pose[1], safe_radius_m=self.MIN_SAFE_RADIUS_M):
                        return False, (f"unsafe arrival: actual pose ({pose[0]:.2f},{pose[1]:.2f}) "
                            f"failed {self.MIN_SAFE_RADIUS_M:.2f}m chassis clearance within "
                            f"{self.settings.arrival_tolerance_m:.2f}m goal tolerance")
                    if last_pose is None or math.dist(pose[:2], last_pose) > 0.03:
                        last_pose, settled_since = pose[:2], now
                    elif settled_since is not None and now - settled_since >= self.settings.arrival_settle_s:
                        return True, "arrival footprint verified"
                else:
                    last_pose = settled_since = None
            else:
                last_pose = settled_since = None
            time.sleep(min(self.NAV_POLL_PERIOD_S, max(0.0, deadline - time.monotonic())))
        return False, "unsafe arrival: no fresh, settled map pose passed chassis-clearance verification"

    def _cancel_navigation(self, run_id: str) -> bool:
        """Confirm the exact navigation run is terminal before another is sent."""
        deadline = time.monotonic() + self.NAV_CANCEL_TIMEOUT_S
        self._mcp_call_sync("cancel", {"run_id": run_id}, timeout_s=min(1.0, self.NAV_CANCEL_TIMEOUT_S))
        while time.monotonic() < deadline:
            result = self._mcp_call_sync("status", {"run_id": run_id},
                timeout_s=min(1.0, max(0.001, deadline - time.monotonic())))
            if result.get("known", True) and str(result.get("state", "")).upper() in (
                    "SUCCEEDED", "FAILED", "CANCELED", "TIMEOUT"):
                return True
            time.sleep(min(0.1, max(0.0, deadline - time.monotonic())))
        log.error("navigation run %s did not confirm cancellation", run_id)
        return False

    def _nav_cancel_rpc(self, run_id: str = "") -> dict:
        """Cancel only the navigation run owned by the active exploration."""
        return self._mcp_call_sync("cancel", {"run_id": run_id})

    def _adjust_safe_radius(self) -> None:
        """Adjust safe radius based on navigation success rate.
        High success rate → smaller radius (more aggressive).
        Low success rate → larger radius (more conservative).
        Only evaluates every RADIUS_EVAL_INTERVAL legs."""
        if self._nav_total_count < self.RADIUS_EVAL_INTERVAL:
            return
        success_rate = self._nav_success_count / self._nav_total_count
        if success_rate > 0.8:
            self._current_safe_radius = max(
                self.MIN_SAFE_RADIUS_M,
                self._current_safe_radius - self.RADIUS_ADJUST_STEP
            )
            log.info("safe radius decreased to %.2f (success rate: %.1f%%)",
                     self._current_safe_radius, success_rate * 100)
        elif success_rate < 0.5:
            self._current_safe_radius = min(
                self.MAX_SAFE_RADIUS_M,
                self._current_safe_radius + self.RADIUS_ADJUST_STEP
            )
            log.info("safe radius increased to %.2f (success rate: %.1f%%)",
                     self._current_safe_radius, success_rate * 100)
        # Reset counters
        self._nav_success_count = 0
        self._nav_total_count = 0

    # ── Glass door detection ─────────────────────────────────────────
    def _update_glass_doors(self) -> None:
        """Query scene service for glass door detections. Updates
        self._glass_doors with list of (x, y) world coordinates.
        Called periodically during exploration loop."""
        if self._scene_endpoint is None:
            return
        # Throttle: only query every 5 seconds
        now = time.time()
        if now - self._last_glass_door_update < 5.0:
            return
        self._last_glass_door_update = now

        try:
            # Call scene service's list_objects MCP tool
            resp = self._mcp_call_sync(
                "list_objects",
                {},
                endpoint=self._scene_endpoint
            )
            if not resp:
                return

            # Extract glass door objects from response
            objects = resp.get("objects", [])
            glass_doors = []
            for obj in objects:
                if obj.get("cls") == "glass_door":
                    pose = obj.get("pose", {})
                    x = pose.get("x", 0.0)
                    y = pose.get("y", 0.0)
                    glass_doors.append((x, y))

            with self._lock:
                self._glass_doors = glass_doors
            if glass_doors:
                log.info("detected %d glass doors", len(glass_doors))
        except Exception as e:  # noqa: BLE001
            log.debug("glass door query failed: %s", e)

    def _is_glass_door_nearby(self, x: float, y: float,
                               radius: float = 1.0) -> bool:
        """Check if a target position is near any detected glass door.
        Used to penalize frontier scores near glass doors."""
        with self._lock:
            glass_doors = list(self._glass_doors)
        for gx, gy in glass_doors:
            dist = ((x - gx) ** 2 + (y - gy) ** 2) ** 0.5
            if dist < radius:
                return True
        return False

    def _get_glass_doors(self) -> list:
        """Return current glass door positions. Thread-safe."""
        with self._lock:
            return list(self._glass_doors)


# ── Helpers ───────────────────────────────────────────────────────────
def _frontiers_left_count(latest_map: Any) -> int:
    if latest_map is None:
        return -1
    try:
        from .frontier import GridView, total_frontier_count
        gv = GridView.from_msg(latest_map)
        return int(total_frontier_count(gv))
    except Exception:
        return -1


def _import_ros() -> dict:
    """Lazy ROS import — keeps unit tests importable without rclpy."""
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import (QoSProfile, ReliabilityPolicy,
                            DurabilityPolicy, HistoryPolicy)
    from rclpy.duration import Duration
    from nav_msgs.msg import OccupancyGrid
    return {
        "rclpy": rclpy,
        "Node": Node,
        "QoSProfile": QoSProfile,
        "ReliabilityPolicy": ReliabilityPolicy,
        "DurabilityPolicy": DurabilityPolicy,
        "HistoryPolicy": HistoryPolicy,
        "Duration": Duration,
        "OccupancyGrid": OccupancyGrid,
    }
