"""Mission state machine — IDLE / TEACH / AUTO / PAUSED / MANUAL / RECOVER / STUCK / DONE.

Owns waypoint sequencing, reach-radius advancement, RECOVER entry on
off-lawn, and resume-from-nearest after MANUAL.
"""
from __future__ import annotations

import math
import threading
from dataclasses import dataclass, field
from enum import Enum

from ..config import ControlCfg
from ..estimator import Pose


class State(str, Enum):
    IDLE = "IDLE"
    TEACH = "TEACH"
    AUTO = "AUTO"
    PAUSED = "PAUSED"
    MANUAL = "MANUAL"
    RECOVER = "RECOVER"
    STUCK = "STUCK"
    DONE = "DONE"


Point = tuple[float, float]


@dataclass
class MissionStatus:
    state: State = State.IDLE
    waypoint_idx: int = 0
    n_waypoints: int = 0
    pass_num: int = 1  # 1 = primary, 2 = crosscut
    coverage_pct: float = 0.0
    distance_m: float = 0.0
    note: str = ""


class Mission:
    def __init__(self, ctrl: ControlCfg):
        self.ctrl = ctrl
        self._lock = threading.Lock()
        self.status = MissionStatus()
        self.path: list[Point] = []
        self._last_pose: Pose | None = None

    # ---- public API -----------------------------------------------------

    def load_path(self, path: list[Point]) -> None:
        with self._lock:
            self.path = list(path)
            self.status.waypoint_idx = 0
            self.status.n_waypoints = len(path)
            self.status.coverage_pct = 0.0

    def start(self) -> None:
        with self._lock:
            if not self.path:
                self.status.note = "no path planned"
                return
            self.status.state = State.AUTO
            self.status.note = ""

    def pause(self) -> None:
        with self._lock:
            if self.status.state in (State.AUTO, State.RECOVER):
                self.status.state = State.PAUSED

    def resume(self) -> None:
        with self._lock:
            if self.status.state in (State.PAUSED, State.MANUAL):
                self._snap_to_nearest_locked()
                self.status.state = State.AUTO
                self.status.note = ""

    def stop(self) -> None:
        with self._lock:
            self.status.state = State.IDLE

    def to_manual(self) -> None:
        with self._lock:
            self.status.state = State.MANUAL

    def to_stuck(self, reason: str = "stuck") -> None:
        with self._lock:
            self.status.state = State.STUCK
            self.status.note = reason

    def to_recover(self) -> None:
        with self._lock:
            self.status.state = State.RECOVER

    def back_to_auto(self) -> None:
        with self._lock:
            if self.status.state == State.RECOVER:
                self.status.state = State.AUTO

    def update(self, pose: Pose) -> tuple[Point | None, bool]:
        """Advance waypoint if within reach. Returns (current_target, done)."""
        with self._lock:
            if not self.path or self.status.state not in (State.AUTO, State.RECOVER):
                return (None, False)
            if self._last_pose is not None:
                self.status.distance_m += math.hypot(
                    pose.x - self._last_pose.x, pose.y - self._last_pose.y
                )
            self._last_pose = pose

            idx = self.status.waypoint_idx
            if idx >= len(self.path):
                self.status.state = State.DONE
                return (None, True)

            target = self.path[idx]
            d = math.hypot(target[0] - pose.x, target[1] - pose.y)
            if d < self.ctrl.reach_m:
                self.status.waypoint_idx = idx + 1
                if self.status.waypoint_idx >= len(self.path):
                    self.status.state = State.DONE
                    return (None, True)
                target = self.path[self.status.waypoint_idx]
            self.status.coverage_pct = 100.0 * self.status.waypoint_idx / max(1, len(self.path))
            return (target, False)

    def snapshot(self) -> MissionStatus:
        with self._lock:
            return MissionStatus(
                state=self.status.state,
                waypoint_idx=self.status.waypoint_idx,
                n_waypoints=self.status.n_waypoints,
                pass_num=self.status.pass_num,
                coverage_pct=self.status.coverage_pct,
                distance_m=self.status.distance_m,
                note=self.status.note,
            )

    # ---- internals ------------------------------------------------------

    def _snap_to_nearest_locked(self) -> None:
        """Pick the nearest not-yet-covered waypoint to the current pose."""
        if not self.path or self._last_pose is None:
            return
        p = self._last_pose
        best_i = self.status.waypoint_idx
        best_d = math.inf
        for i in range(self.status.waypoint_idx, len(self.path)):
            wp = self.path[i]
            d = math.hypot(wp[0] - p.x, wp[1] - p.y)
            if d < best_d:
                best_d = d
                best_i = i
        self.status.waypoint_idx = best_i
