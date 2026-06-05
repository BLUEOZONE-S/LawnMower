"""Fused pose estimator — complementary filter.

Dead-reckons (x, y, θ) every tick using IMU yaw and wheel-odometry Δs.
Whenever a fresh GPS fix lands, the estimator snaps the position toward
the fix by a fixed blend factor ALPHA (same shape as the validated sim's
ALPHA blend).

This is intentionally simple — an EKF is an upgrade path. The brief calls
this out: complementary filter now, EKF later if needed.
"""
from __future__ import annotations

import math
import threading
import time
from dataclasses import dataclass

from .config import EstimatorCfg
from .gnss.lc29h import GnssFix
from .nav.geo import Origin, to_enu


@dataclass
class Pose:
    x: float = 0.0  # east, m
    y: float = 0.0  # north, m
    theta: float = 0.0  # yaw, rad (0 = +x = east, CCW positive)
    last_update_mono: float = 0.0


class Estimator:
    def __init__(self, cfg: EstimatorCfg, origin: Origin):
        self.cfg = cfg
        self.origin = origin
        self._lock = threading.Lock()
        self.pose = Pose(last_update_mono=time.monotonic())
        self._last_gps_mono = 0.0
        self.gps_age_s = math.inf
        self.last_fix_quality = 0
        self._yaw_offset = math.radians(cfg.imu_yaw_offset_deg)

    # ---- public API -----------------------------------------------------

    def seed(self, x: float, y: float, theta: float = 0.0) -> None:
        with self._lock:
            self.pose.x = x
            self.pose.y = y
            self.pose.theta = theta
            self.pose.last_update_mono = time.monotonic()

    def dead_reckon(self, yaw_rad: float, ds_m: float) -> Pose:
        """Update pose given current IMU yaw + Δs (meters traveled since last call)."""
        with self._lock:
            theta = yaw_rad + self._yaw_offset
            self.pose.x += ds_m * math.cos(theta)
            self.pose.y += ds_m * math.sin(theta)
            self.pose.theta = theta
            self.pose.last_update_mono = time.monotonic()
            return self._snapshot()

    def ingest_gps(self, fix: GnssFix) -> None:
        """Blend the fused position toward a fresh GPS fix."""
        gx, gy = to_enu(fix.lat, fix.lon, self.origin)
        alpha = self.cfg.gps_blend_alpha
        with self._lock:
            self.pose.x = (1 - alpha) * self.pose.x + alpha * gx
            self.pose.y = (1 - alpha) * self.pose.y + alpha * gy
            self._last_gps_mono = time.monotonic()
            self.gps_age_s = 0.0
            self.last_fix_quality = fix.quality

    def tick_age(self) -> None:
        """Refresh gps_age_s for safety checks. Cheap; call on the safety thread."""
        with self._lock:
            self.gps_age_s = time.monotonic() - self._last_gps_mono if self._last_gps_mono else math.inf

    def snapshot(self) -> Pose:
        with self._lock:
            return self._snapshot()

    # ---- internals ------------------------------------------------------

    def _snapshot(self) -> Pose:
        return Pose(
            x=self.pose.x,
            y=self.pose.y,
            theta=self.pose.theta,
            last_update_mono=self.pose.last_update_mono,
        )
