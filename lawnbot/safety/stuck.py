"""Stuck detection with the hard 60 s ceiling.

Per brief §13:
  - Only counts while it SHOULD be moving (AUTO/RECOVER with commanded v>0).
  - Window: net fused-position displacement over `window_s`.
  - If displacement < min_progress_m → progress stall.
  - Faster check: odom moving while GPS isn't = slip / blocked.
  - On stall, attempt a bounded recovery up to `recovery_tries` times
    (the recovery maneuver itself happens in mission/RECOVER).
  - Hard ceiling: total stuck time `giveup_s` (60 s by spec) → STUCK,
    motors stop, wait for the operator.
"""
from __future__ import annotations

import math
import time
from collections import deque
from dataclasses import dataclass

from ..config import StuckCfg
from ..estimator import Pose


@dataclass
class StuckOutcome:
    state: str  # "ok" | "recover" | "stuck"
    reason: str = ""
    recovery_count: int = 0
    seconds_in_state: float = 0.0


class StuckDetector:
    def __init__(self, cfg: StuckCfg):
        self.cfg = cfg
        self._poses: deque[tuple[float, float, float]] = deque()  # (t, x, y)
        self._recovery_count = 0
        self._stall_started: float | None = None
        self._cum_odom = 0.0
        self._cum_gps = 0.0

    def reset(self) -> None:
        self._poses.clear()
        self._recovery_count = 0
        self._stall_started = None
        self._cum_odom = 0.0
        self._cum_gps = 0.0

    def update(self, pose: Pose, commanding_motion: bool, odom_delta_m: float) -> StuckOutcome:
        """Call every control tick (or every safety tick).

        commanding_motion: True if the controller is sending v>0 right now and
        the mission is in AUTO or RECOVER. We never count stalls in
        PAUSED/MANUAL/awaiting-fix.
        """
        now = time.monotonic()
        if not commanding_motion:
            self._stall_started = None
            self._poses.clear()
            self._cum_odom = 0.0
            self._cum_gps = 0.0
            return StuckOutcome(state="ok")

        self._poses.append((now, pose.x, pose.y))
        # Trim window.
        cutoff = now - self.cfg.window_s
        while self._poses and self._poses[0][0] < cutoff:
            self._poses.popleft()

        # Cumulative GPS distance in the window.
        gps_disp = 0.0
        if len(self._poses) >= 2:
            (_, x0, y0) = self._poses[0]
            (_, x1, y1) = self._poses[-1]
            gps_disp = math.hypot(x1 - x0, y1 - y0)
        self._cum_odom += odom_delta_m

        # Mismatch check (cheap, triggers sooner than the window).
        mismatch = self._cum_odom > 0.5 and gps_disp < 0.15
        progress_stall = (
            len(self._poses) >= 5
            and (now - self._poses[0][0]) >= 0.9 * self.cfg.window_s
            and gps_disp < self.cfg.min_progress_m
        )

        if not (progress_stall or mismatch):
            self._stall_started = None
            return StuckOutcome(state="ok")

        if self._stall_started is None:
            self._stall_started = now
        elapsed = now - self._stall_started

        if elapsed >= self.cfg.giveup_s:
            return StuckOutcome(state="stuck", reason="60s ceiling", seconds_in_state=elapsed)

        if self._recovery_count < self.cfg.recovery_tries:
            self._recovery_count += 1
            return StuckOutcome(
                state="recover",
                reason="progress stall" if progress_stall else "odom/gps mismatch",
                recovery_count=self._recovery_count,
                seconds_in_state=elapsed,
            )

        return StuckOutcome(state="stuck", reason="recovery exhausted", seconds_in_state=elapsed)
