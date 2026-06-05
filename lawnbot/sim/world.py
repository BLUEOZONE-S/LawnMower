"""Virtual Ackermann world — the physics simulator.

Owns the ground-truth rover state ``(x, y, theta, v)`` in local ENU meters.
A daemon thread integrates the bicycle model at a fixed dt; sim drivers
poke ``set_duty`` / ``set_steer`` and pull noisy ``true_pose`` / odometry
out the other side.

Coordinate frame matches the rest of the stack: +x = east, +y = north,
theta = yaw CCW from +x. The world's ``origin`` is the lat/lon anchor; the
sim GPS converts ENU back to lat/lon through it so the estimator sees a
realistic ``GnssFix`` stream.
"""
from __future__ import annotations

import math
import random
import threading
import time
from dataclasses import dataclass

from ..config import DriveCfg, GeometryCfg, SimCfg
from ..nav.geo import Origin


@dataclass
class WorldState:
    x: float
    y: float
    theta: float
    v: float
    duty: float
    steer: float


class SimWorld:
    """Ground-truth rover state + physics tick.

    Thread-safety: callers may set actuator commands and read state from any
    thread; the physics thread holds the lock for its integration step.
    """

    PHYS_HZ = 100  # internal integration rate

    def __init__(
        self,
        sim_cfg: SimCfg,
        geom: GeometryCfg,
        drive: DriveCfg,
        origin: Origin,
    ):
        self.cfg = sim_cfg
        self.geom = geom
        self.drive = drive
        self.origin = origin

        self._lock = threading.Lock()
        self._x = float(sim_cfg.start_x)
        self._y = float(sim_cfg.start_y)
        self._theta = math.radians(sim_cfg.start_theta_deg)
        self._v = 0.0
        self._duty = 0.0
        self._steer = 0.0
        self._ds_accum = 0.0  # signed odometry since last read
        self._battery_pct = float(sim_cfg.battery_start_pct)
        # time_scale > 1 makes the simulated world run faster than wall-clock:
        # each physics tick still happens on the real 100 Hz wall clock, but it
        # integrates by scale·dt of sim time. Sim drivers (GPS) shrink their
        # publishing period in proportion so cadence stays the same per sim-sec.
        self._time_scale = 1.0

        self.rng = random.Random(sim_cfg.seed)

        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True, name="sim-world")
        self._thread.start()

    # ---- actuator interface (called by sim drivers) -------------------

    def set_duty(self, duty: float) -> None:
        with self._lock:
            self._duty = max(-1.0, min(1.0, float(duty)))

    def set_steer(self, delta_rad: float) -> None:
        cap = self.geom.steer_max_rad
        with self._lock:
            self._steer = max(-cap, min(cap, float(delta_rad)))

    # ---- teleport / reset --------------------------------------------

    def set_pose(self, x: float, y: float, theta_rad: float) -> None:
        """Hard-teleport the rover. Zeros velocity + odom accumulator so the
        controller and estimator start from a clean slate after the jump."""
        with self._lock:
            self._x = float(x)
            self._y = float(y)
            self._theta = math.atan2(math.sin(theta_rad), math.cos(theta_rad))
            self._v = 0.0
            self._duty = 0.0
            self._steer = 0.0
            self._ds_accum = 0.0

    # ---- sensor interface (called by sim drivers) ---------------------

    def true_pose(self) -> tuple[float, float, float]:
        with self._lock:
            return self._x, self._y, self._theta

    def state(self) -> WorldState:
        with self._lock:
            return WorldState(
                x=self._x, y=self._y, theta=self._theta,
                v=self._v, duty=self._duty, steer=self._steer,
            )

    def read_odom_delta(self) -> float:
        """Pop the signed distance traveled since the last call."""
        with self._lock:
            ds = self._ds_accum
            self._ds_accum = 0.0
            return ds

    def battery_percent(self) -> float:
        with self._lock:
            return self._battery_pct

    # ---- time-scale (wall-clock speedup) ------------------------------

    def set_time_scale(self, scale: float) -> None:
        """Run the simulated world at scale× wall-clock. Range [0.1, 10]."""
        with self._lock:
            self._time_scale = max(0.1, min(10.0, float(scale)))

    @property
    def time_scale(self) -> float:
        with self._lock:
            return self._time_scale

    # ---- lifecycle ----------------------------------------------------

    def stop(self) -> None:
        self._stop.set()
        self._thread.join(timeout=1.0)

    # ---- internals ----------------------------------------------------

    def _run(self) -> None:
        wall_dt = 1.0 / self.PHYS_HZ
        v_max = float(self.cfg.world_v_max)
        tau = max(1e-3, float(self.cfg.motor_tau_s))
        L = max(1e-3, self.geom.wheelbase_m)
        drain_per_s = float(self.cfg.battery_drain_pct_per_min) / 60.0

        next_tick = time.monotonic()
        while not self._stop.is_set():
            with self._lock:
                scale = self._time_scale
                # Sim-time step. At scale=1, sim and wall time match.
                # At scale=5, each wall tick (10 ms) integrates 50 ms of sim
                # time — the bicycle model is linear in v/omega so larger dt
                # stays numerically stable up to ~100 ms.
                dt = wall_dt * scale

                # First-order motor lag toward the duty-implied velocity.
                v_target = self._duty * v_max
                self._v += (v_target - self._v) * (dt / tau)

                # Bicycle model integration.
                omega = self._v * math.tan(self._steer) / L
                self._x += self._v * math.cos(self._theta) * dt
                self._y += self._v * math.sin(self._theta) * dt
                self._theta = math.atan2(
                    math.sin(self._theta + omega * dt),
                    math.cos(self._theta + omega * dt),
                )

                # Wheel-encoder odometry: signed distance.
                self._ds_accum += self._v * dt

                # Battery drains in sim time too — at 10×, an "hour" passes in
                # 6 minutes of wall time, so the discharge curve scales with it.
                self._battery_pct = max(0.0, self._battery_pct - drain_per_s * dt)

            next_tick += wall_dt
            slack = next_tick - time.monotonic()
            if slack > 0:
                time.sleep(slack)
            else:
                # Falling behind — resync clock to avoid spiral of death.
                next_tick = time.monotonic()
