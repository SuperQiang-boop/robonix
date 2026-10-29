"""Lifecycle state and bounded cancellable task scheduling."""
from __future__ import annotations

import math
import logging
import copy
import re
import threading
import time
import uuid
from dataclasses import asdict, dataclass

from .settings import Settings

BACKEND_UNAVAILABLE = "relocalization task backend is not ready; no pose has been verified"


@dataclass(frozen=True)
class VerifiedResult:
    """Internal handoff receipt whose reader rechecks live map and scan evidence."""

    current: object


@dataclass(frozen=True)
class Status:
    process_id: str
    request_id: str = ""
    active: bool = False
    busy: bool = False
    localization_valid: bool = False
    map_id: str = ""
    generation: int = 0
    database_sha256: str = ""
    published_stamp_ns: int = 0
    observation_stamp_ns: int = 0
    x: float = 0.0
    y: float = 0.0
    theta: float = 0.0
    residual_m: float = -1.0
    overlap: float = 0.0
    ambiguous: bool = False
    detail: str = BACKEND_UNAVAILABLE


class Runtime:
    """Schedule one backend task while keeping candidate separate from validity."""

    def __init__(self, process_id: str | None = None):
        """Own task/lifecycle locks and keep no backend until activation."""
        self.process_id = process_id or uuid.uuid4().hex
        self._active = False
        self._settings = None
        self._lock = threading.RLock()
        self._lifecycle_lock = threading.RLock()
        self._task_runner = None
        self._task = None
        self._last = None
        self._candidate = None
        self._deactivating = False

    def activate(self, settings: Settings, task_runner=None) -> None:
        """Enable request handling with an optional fully verified task backend."""
        if task_runner is not None and not callable(task_runner):
            raise TypeError("task backend must be callable")
        with self._lifecycle_lock, self._lock:
            if self._active or self._deactivating:
                raise RuntimeError("service is already active")
            self._settings = settings
            self._task_runner = task_runner
            self._task = None
            self._last = None
            self._clear_candidate()
            self._active = True

    def deactivate(self) -> None:
        """Cancel and join active work before invalidating this service instance."""
        with self._lifecycle_lock:
            self._deactivate()

    def _deactivate(self) -> None:
        """Serialize shutdown; retain timed-out workers so cleanup can be retried."""
        with self._lock:
            task = self._task
            self._active = False
            if self._candidate is not None:
                self._last["detail"] = "service deactivated; candidate discarded"
            self._clear_candidate()
            self._deactivating = task is not None
            if task is not None:
                task["token"].cancel()
                worker = task["thread"]
            else:
                worker = None
        if worker is not None:
            worker.join(timeout=5)
            if worker.is_alive():
                raise RuntimeError("relocalization worker did not stop after cancellation")
        with self._lock:
            if self._task is task:
                self._task = None
            self._task_runner = None
            self._deactivating = False

    def status(self, stamp_ns: int = 0) -> dict:
        """Expose validity only while a verified handoff has fresh live evidence."""
        with self._lock:
            if self._active and self._candidate is not None:
                saved = self._candidate
                if isinstance(saved['result'], VerifiedResult):
                    try:
                        result = saved['result'].current()
                        if saved['task']['token'].cancelled:
                            raise RuntimeError('localization request cancelled')
                        return asdict(Status(process_id=self.process_id, active=True,
                            request_id=saved['task']['request_id'], localization_valid=True,
                            published_stamp_ns=stamp_ns, **saved['task']['identity'],
                            observation_stamp_ns=result.observation_stamp_ns,
                            x=result.x, y=result.y, theta=result.yaw,
                            residual_m=result.rmse_m, overlap=result.overlap,
                            detail='mapping pose verified against fresh independent lidar observations'))
                    except Exception as exc:
                        self._last['detail'] = f'localization invalid: {exc}'
                        self._clear_candidate()
            task = self._task or self._last
            if task is None:
                return asdict(Status(process_id=self.process_id, active=self._active,
                                     published_stamp_ns=stamp_ns))
            return asdict(Status(process_id=self.process_id, active=self._active,
                request_id=task["request_id"], busy=self._task is not None,
                published_stamp_ns=stamp_ns, detail=task["detail"]))

    def set_detail(self, detail: str) -> None:
        """Publish readiness work or failure without claiming a localization request."""
        if not isinstance(detail, str):
            raise TypeError("status detail must be a string")
        with self._lock:
            if self._task is not None or self._candidate is not None:
                return
            if detail:
                self._last = {"request_id": "", "detail": detail}
            else:
                self._last = None

    def relocalize(self, map_id: str, generation: int, database_sha256: str,
                   timeout_s: float) -> dict:
        """Validate and asynchronously schedule one request when backend is ready."""
        with self._lock:
            if not self._active:
                raise RuntimeError("relocalization service is not active")
            if self._settings.map_id and map_id != self._settings.map_id:
                raise ValueError("requested map_id differs from the configured map")
            if type(generation) is not int or not 0 <= generation < 2**64:
                raise ValueError("generation must be a uint64")
            if not isinstance(database_sha256, str) or not re.fullmatch(r"[0-9a-f]{64}", database_sha256):
                raise ValueError("database_sha256 must be a lowercase SHA-256 hex digest")
            if type(timeout_s) not in (int, float) or not math.isfinite(timeout_s) or not 0 <= timeout_s <= 120:
                raise ValueError("timeout_s must be finite and in [0, 120]")
            if self._task_runner is None:
                return {"accepted": False, "request_id": "", "detail": BACKEND_UNAVAILABLE}
            if self._task is not None:
                return {"accepted": False, "request_id": "", "detail": "another relocalization request is already active"}
            if timeout_s == 0:
                timeout_s = self._settings.timeout_s
            self._clear_candidate()
            request_id = uuid.uuid4().hex
            from _relocalization_bbs import CancellationToken
            task = {"request_id": request_id, "token": CancellationToken(),
                    "identity": dict(map_id=map_id, generation=generation,
                                     database_sha256=database_sha256),
                    "deadline": time.monotonic()+timeout_s, "timed_out": False,
                    "detail": "relocalization request accepted; localization is not yet verified"}
            task["timer"] = threading.Timer(timeout_s, self._expire, args=(task,))
            task["timer"].daemon = True
            worker = threading.Thread(target=self._run_task,
                args=(task, map_id, generation, database_sha256),
                name=f"relocalization-{request_id[:8]}", daemon=True)
            task["thread"] = worker
            self._task = task
            try:
                task["timer"].start()
                worker.start()
            except Exception:
                task["timer"].cancel()
                self._task = None
                raise
            return {"accepted": True, "request_id": request_id, "detail": task["detail"]}

    def _expire(self, task):
        """Cancel running work or discard its retained candidate at the deadline."""
        with self._lock:
            if self._task is task:
                task["timed_out"] = True
                task["token"].cancel()
                task["detail"] = "relocalization deadline exceeded; waiting for worker to stop"
            elif self._candidate is not None and self._candidate["task"] is task:
                self._clear_candidate()
                self._last["detail"] = "relocalization deadline exceeded; candidate discarded"

    def _run_task(self, task, map_id, generation, digest):
        """Run one injected backend and publish only truthful geometric progress."""
        result = None
        try:
            remaining = task["deadline"]-time.monotonic()
            if remaining <= 0 or task["token"].cancelled:
                raise TimeoutError("request expired or cancelled before computation")
            result = self._task_runner(map_id, generation, digest, remaining, task['token'])
            if isinstance(result, VerifiedResult):
                result.current()
                detail = 'mapping handoff verified; ongoing lidar verification enabled'
            else:
                result = copy.deepcopy(result)
                detail = ('geometric candidate computed; pose handoff and verification are required'
                          if result is not None else 'relocalization produced no candidate')
        except Exception as exc:
            detail = f"relocalization failed: {type(exc).__name__}: {exc}"
            logging.warning("request %s: %s", task["request_id"], detail, exc_info=True)
        with self._lock:
            if self._task is task:
                if task["timed_out"] or time.monotonic() >= task["deadline"]:
                    detail = "relocalization deadline exceeded; candidate discarded"
                elif task.get("invalidated"):
                    detail = task["invalidated"]
                elif task.get("cancel_requested"):
                    detail = "relocalization request cancelled; localization is invalid"
                elif self._active and result is not None:
                    self._candidate = {"task": task, "result": result}
                if self._candidate is None or isinstance(result, VerifiedResult):
                    task["timer"].cancel()
                task["detail"] = detail
                self._last = {"request_id": task["request_id"], "detail": detail,
                              "cancelled": task["token"].cancelled}
                self._task = None

    def _clear_candidate(self):
        """Discard stored geometry and its timer; caller must hold the task lock."""
        if self._candidate is not None:
            self._candidate["task"]["timer"].cancel()
            if isinstance(self._candidate['result'], VerifiedResult):
                self._candidate['task']['token'].cancel()
            self._candidate = None

    def _candidate_for_handoff(self, request_id):
        """Copy unverified geometry for internal inspection, never authorize a pose.

        This is not a handoff operation or a validity check. The future handoff
        must revalidate mapping identity, continuous stationary observations,
        freshness and the request immediately before any pose injection. A copy
        may be invalidated as soon as this method releases the lock.
        """
        with self._lock:
            if not self._active or self._candidate is None:
                return None
            if isinstance(self._candidate['result'], VerifiedResult):
                return None
            task = self._candidate["task"]
            if time.monotonic() >= task["deadline"]:
                self._expire(task)
                return None
            if task["request_id"] != request_id or task["token"].cancelled:
                return None
            return dict(request_id=request_id, identity=copy.deepcopy(task["identity"]),
                        deadline=task["deadline"],
                        result=copy.deepcopy(self._candidate["result"]))

    def _invalidate(self, detail):
        """Latch an internal dependency failure until a new request is scheduled.

        A future lifecycle observer must call this before a mapping transition
        and on provider/sensor loss. Same-map restoration cannot revive the
        cancelled work or discarded geometry. Production observation is not
        connected yet; this hook alone does not prove continuous map identity.
        """
        if not isinstance(detail, str) or not detail.strip():
            raise ValueError("invalidation detail must be nonempty")
        with self._lock:
            self._clear_candidate()
            task = self._task or self._last
            if task is not None:
                if self._task is not None:
                    task["invalidated"] = detail
                    task["token"].cancel()
                task["detail"] = detail

    def cancel(self, request_id: str) -> dict:
        """Cancel only the active matching request; stale IDs cannot affect work."""
        if not isinstance(request_id, str) or not request_id.strip():
            raise ValueError("request_id must be nonempty")
        with self._lock:
            if (self._candidate is not None and
                    self._candidate["task"]["request_id"] == request_id):
                self._clear_candidate()
                self._last["cancelled"] = True
                self._last["detail"] = "relocalization request cancelled; candidate discarded"
                return {"ok": True, "detail": self._last["detail"]}
            if (self._last is not None and self._last["request_id"] == request_id
                    and self._last["cancelled"]):
                return {"ok": True, "detail": "matching request has already been cancelled"}
            if self._task is None or self._task["request_id"] != request_id:
                return {"ok": False, "detail": "no active request matches request_id"}
            self._task["cancel_requested"] = True
            self._task["token"].cancel()
            self._task["detail"] = "relocalization request cancelled; localization is invalid"
            return {"ok": True, "detail": self._task["detail"]}
