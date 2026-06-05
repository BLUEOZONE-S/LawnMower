"""Safety monitor — battery, geofence, RTK age, watchdog, stop command.

Runs at ≥10 Hz on its own thread, independent of nav. Any trip → request
motors stop and set a state flag the main loop can read.

This is a watchdog, not a kill chain — it asks the drive layer to stop. The
motor_hat command-timeout enforces the actual cutoff in hardware-time if
the loop or this monitor itself dies.
"""
from __future__ import annotations

import threading
import time
from dataclasses import dataclass

from ..config import SafetyCfg
from ..estimator import Estimator
from ..nav.geometry import Polygon, point_in_polygon
from ..power.pisugar import PiSugar


@dataclass
class SafetyState:
    armed: bool = True
    battery_pct: float = -1.0
    charging: bool = False
    gps_age_s: float = float("inf")
    fix_quality: int = 0
    pose_inside_geofence: bool = True
    watchdog_ok: bool = True
    last_trip: str = ""


class SafetyMonitor:
    def __init__(
        self,
        cfg: SafetyCfg,
        min_fix_quality: int,
        estimator: Estimator,
        pisugar: PiSugar | None,
        boundary_provider,
    ):
        """boundary_provider() returns the current boundary polygon (or None)
        in local ENU meters. Provider is a callable so the geofence can update
        when a new boundary is taught.
        """
        self.cfg = cfg
        self.min_fix_quality = min_fix_quality
        self.est = estimator
        self.pi = pisugar
        self._boundary_provider = boundary_provider

        self._lock = threading.Lock()
        self.state = SafetyState()
        self._last_pet_mono = time.monotonic()
        self._degrade_since: float | None = None

        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._pet_lock = threading.Lock()

        self.on_trip = lambda reason: None  # callback: (reason: str) -> None

    # ---- public API -----------------------------------------------------

    def start(self) -> None:
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        self._thread.join(timeout=2.0)

    def pet(self) -> None:
        """Control loop calls this every tick. Late = watchdog trip."""
        with self._pet_lock:
            self._last_pet_mono = time.monotonic()

    def request_stop(self, reason: str) -> None:
        with self._lock:
            self.state.armed = False
            self.state.last_trip = reason
        self.on_trip(reason)

    def rearm(self) -> None:
        with self._lock:
            self.state.armed = True
            self.state.last_trip = ""

    def snapshot(self) -> SafetyState:
        with self._lock:
            return SafetyState(**self.state.__dict__)

    # ---- internals ------------------------------------------------------

    def _run(self) -> None:
        while not self._stop.is_set():
            self._check_once()
            time.sleep(0.1)

    def _check_once(self) -> None:
        now = time.monotonic()

        # Battery
        if self.pi is not None:
            bs = self.pi.poll()
            with self._lock:
                self.state.battery_pct = bs.percent
                self.state.charging = bs.charging
            if bs.available and not bs.charging:
                if bs.percent < self.cfg.stop_pct:
                    self.request_stop(f"battery {bs.percent:.0f}% < {self.cfg.stop_pct}%")
                    return

        # GPS
        self.est.tick_age()
        gps_age = self.est.gps_age_s
        fix_q = self.est.last_fix_quality
        with self._lock:
            self.state.gps_age_s = gps_age
            self.state.fix_quality = fix_q
        degraded = (gps_age > self.cfg.degrade_sec) or (fix_q and fix_q < self.min_fix_quality)
        if degraded:
            if self._degrade_since is None:
                self._degrade_since = now
            elif (now - self._degrade_since) > self.cfg.degrade_sec:
                self.request_stop(f"RTK degraded for >{self.cfg.degrade_sec}s")
                return
        else:
            self._degrade_since = None

        # Geofence
        boundary = self._boundary_provider()
        if boundary is not None:
            pose = self.est.snapshot()
            inside = point_in_polygon((pose.x, pose.y), boundary)
            with self._lock:
                self.state.pose_inside_geofence = inside
            if not inside:
                self.request_stop("geofence breach")
                return

        # Watchdog: control loop heartbeat.
        with self._pet_lock:
            pet_age = now - self._last_pet_mono
        wd_ok = pet_age < 1.0
        with self._lock:
            self.state.watchdog_ok = wd_ok
        if not wd_ok:
            self.request_stop("control-loop watchdog stall")
