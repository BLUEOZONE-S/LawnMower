"""Sim drivers — drop-in replacements for the hardware leaf modules.

Each class mirrors the public surface of its hardware counterpart so the
rest of the stack (estimator, controller, planner, mission, safety, teleop,
UI) is bit-identical between sim and real runs.

Shared state lives in a :class:`SimWorld` that all drivers reference.
"""
from __future__ import annotations

import math
import threading
import time

from ..config import DriveCfg, GeometryCfg, GnssCfg, SimCfg, SteeringCfg
from ..gnss.lc29h import (
    CONSTELLATION,
    FIX_RTK_FIXED,
    GnssFix,
    SatelliteInfo,
    SatelliteSnapshot,
)
from ..nav.geo import Origin, to_ll
from ..power.pisugar import BatteryState
from .world import SimWorld


# ---- motor HAT ---------------------------------------------------------

class SimMotorHAT:
    """Stand-in for :class:`lawnbot.drive.motor_hat.MotorHAT`.

    Mirrors the public API, including the command-timeout watchdog that
    auto-stops the motors if the control loop dies.
    """

    def __init__(self, world: SimWorld, cfg: DriveCfg, timeout_ms: int = 300):
        self.cfg = cfg
        self.world = world
        self.timeout_s = timeout_ms / 1000.0

        self._lock = threading.Lock()
        self._last_cmd = time.monotonic()
        self._duty = 0.0

        self._wd_stop = threading.Event()
        self._wd_thread = threading.Thread(target=self._watchdog, daemon=True, name="sim-motor-wd")
        self._wd_thread.start()
        self.stop()

    def set_throttle(self, duty: float) -> None:
        d = max(-1.0, min(1.0, float(duty)))
        with self._lock:
            self._last_cmd = time.monotonic()
            self._duty = d
        self.world.set_duty(d)

    def set_each(self, rear: float, front: float) -> None:
        # Sim is one throttle; average the two so motor_calibrate.py works in sim.
        self.set_throttle(0.5 * (float(rear) + float(front)))

    def stop(self) -> None:
        with self._lock:
            self._duty = 0.0
        self.world.set_duty(0.0)

    def brake(self) -> None:
        self.stop()

    def close(self) -> None:
        self._wd_stop.set()
        self.stop()

    @property
    def state(self) -> dict:
        with self._lock:
            duty = self._duty
            age = time.monotonic() - self._last_cmd
        forward = duty >= 0
        mag = abs(duty)
        return {
            "rear": {"duty": mag, "forward": forward},
            "front": {"duty": mag, "forward": forward},
            "age_s": age,
        }

    def _watchdog(self) -> None:
        while not self._wd_stop.is_set():
            with self._lock:
                stale = (time.monotonic() - self._last_cmd) > self.timeout_s
                moving = abs(self._duty) > 1e-4
            if stale and moving:
                self.stop()
            time.sleep(0.05)


# ---- steering servo ----------------------------------------------------

class SimSteeringServo:
    """Stand-in for :class:`lawnbot.drive.servo.SteeringServo`."""

    def __init__(self, world: SimWorld, cfg: SteeringCfg, geom: GeometryCfg):
        self.world = world
        self.cfg = cfg
        self.geom = geom
        self.center()

    def set_steer(self, delta_rad: float) -> None:
        d = -delta_rad if self.cfg.invert else delta_rad
        self.world.set_steer(d)

    def set_us(self, us: int) -> None:
        # Inverse of servo.set_steer's pulse-width mapping (asymmetric).
        us = max(self.cfg.us_min, min(self.cfg.us_max, int(us)))
        dmax = self.geom.steer_max_rad
        if us >= self.cfg.us_center:
            span = self.cfg.us_max - self.cfg.us_center or 1
            delta = (us - self.cfg.us_center) / span * dmax
        else:
            span = self.cfg.us_center - self.cfg.us_min or 1
            delta = (us - self.cfg.us_center) / span * dmax
        self.set_steer(delta)

    def center(self) -> None:
        self.world.set_steer(0.0)

    def release(self) -> None:
        self.world.set_steer(0.0)

    def close(self) -> None:
        self.release()


# ---- GPS ---------------------------------------------------------------

class SimLC29H:
    """Stand-in for :class:`lawnbot.gnss.lc29h.LC29H`.

    Emits one :class:`GnssFix` per ``sim.gps_period_s`` from the world pose
    plus Gaussian noise. Also synthesizes a stable multi-constellation
    skyplot (~16 satellites) so the debug UI has real-looking data.
    ``write_rtcm`` is a no-op; ``close`` stops the publishing thread.
    """

    # Talker IDs roughly match what a multi-GNSS LC29H emits in the wild.
    _CONSTELLATIONS = [
        # (talker, prn-base, count, used-share)
        ("GP", 1, 8, 0.85),    # GPS
        ("GL", 65, 5, 0.7),    # GLONASS
        ("GA", 1, 4, 0.7),     # Galileo
        ("GB", 1, 3, 0.6),     # BeiDou
    ]

    def __init__(self, world: SimWorld, cfg: GnssCfg, sim_cfg: SimCfg, origin: Origin):
        self.world = world
        self.cfg = cfg
        self.sim_cfg = sim_cfg
        self.origin = origin

        self._lock = threading.Lock()
        self._latest: GnssFix | None = None
        self._gga_count = 0
        self._gsv_count = 0
        self._sats: list[SatelliteInfo] = []
        self._constellation = self._seed_constellation()
        self._t0 = time.monotonic()

        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True, name="sim-gps")
        self._thread.start()

    @property
    def latest(self) -> GnssFix | None:
        with self._lock:
            return self._latest

    @property
    def satellites(self) -> SatelliteSnapshot:
        with self._lock:
            return SatelliteSnapshot(
                sats=[SatelliteInfo(**s.__dict__) for s in self._sats],
                timestamp_mono=time.monotonic(),
            )

    @property
    def stats(self) -> dict:
        with self._lock:
            return {
                "raw_lines": self._gga_count,
                "gga_lines": self._gga_count,
                "gsv_lines": self._gsv_count,
                "last_quality": self._latest.quality if self._latest else None,
                "last_age_s": self._latest.age_s if self._latest else None,
            }

    def write_rtcm(self, _data: bytes) -> None:
        # No corrections needed in sim; absorb the bytes silently.
        return

    def close(self) -> None:
        self._stop.set()
        self._thread.join(timeout=1.0)

    # ---- internals ----------------------------------------------------

    def _seed_constellation(self) -> list[dict]:
        """Spread satellites evenly in azimuth, with a believable elevation mix.

        Each entry keeps the per-satellite parameters that drive the slow
        animation in :meth:`_update_sats`; the public :class:`SatelliteInfo`
        objects are derived from these every cycle.
        """
        rng = self.world.rng
        sats: list[dict] = []
        for talker, base, count, used_share in self._CONSTELLATIONS:
            for i in range(count):
                # Roughly even azimuth distribution per constellation, jittered.
                az0 = (360.0 * i / max(1, count)) + rng.uniform(-15, 15)
                # Drift slowly clockwise (~6 deg / 5 min) — looks alive without rushing.
                drift = rng.uniform(0.018, 0.045)  # deg/s
                # Elevation 15..80°, weighted away from horizon.
                el0 = rng.triangular(15.0, 80.0, 55.0)
                # SNR baseline scales with elevation (zenith ~48, horizon ~24).
                base_snr = 24.0 + (el0 / 90.0) * 24.0 + rng.uniform(-2.5, 2.5)
                used = rng.random() < used_share
                sats.append({
                    "talker": talker,
                    "prn": base + i,
                    "constellation": CONSTELLATION.get(talker, talker),
                    "az0": az0,
                    "drift": drift,
                    "el0": el0,
                    "el_amp": rng.uniform(2.0, 6.0),
                    "el_freq": rng.uniform(0.005, 0.02),  # rad/s
                    "el_phase": rng.uniform(0.0, 6.283),
                    "snr_base": base_snr,
                    "snr_jitter_amp": rng.uniform(0.8, 2.5),
                    "snr_jitter_freq": rng.uniform(0.5, 2.0),
                    "snr_jitter_phase": rng.uniform(0.0, 6.283),
                    "used": used,
                })
        return sats

    def _update_sats(self) -> None:
        """Re-derive SatelliteInfo from the slow-moving constellation model."""
        t = time.monotonic() - self._t0
        rng = self.world.rng
        new: list[SatelliteInfo] = []
        for s in self._constellation:
            az = (s["az0"] + s["drift"] * t) % 360.0
            el = max(0.0, min(90.0, s["el0"] + s["el_amp"] * math.sin(
                s["el_freq"] * t + s["el_phase"])))
            snr = s["snr_base"] + s["snr_jitter_amp"] * math.sin(
                s["snr_jitter_freq"] * t + s["snr_jitter_phase"]
            ) + rng.uniform(-0.5, 0.5)
            # Satellites below horizon stop tracking.
            if el < 2.0:
                snr = 0.0
            new.append(SatelliteInfo(
                prn=s["prn"],
                talker=s["talker"],
                constellation=s["constellation"],
                elevation_deg=el,
                azimuth_deg=az,
                snr_dbhz=max(0.0, snr),
                used=bool(s["used"] and snr > 18.0),
            ))
        with self._lock:
            self._sats = new
            self._gsv_count += 1

    def _run(self) -> None:
        nominal_period = max(0.05, float(self.sim_cfg.gps_period_s))
        noise = max(0.0, float(self.sim_cfg.gps_noise_m))
        rng = self.world.rng
        # Seed the satellites once before the first fix lands so the UI has
        # data on the very first snapshot.
        self._update_sats()
        next_tick = time.monotonic()
        while not self._stop.is_set():
            x, y, _ = self.world.true_pose()
            nx = x + rng.gauss(0.0, noise)
            ny = y + rng.gauss(0.0, noise)
            lat, lon = to_ll(nx, ny, self.origin)
            self._update_sats()
            tracked = sum(1 for s in self._sats if s.snr_dbhz > 0)
            used = sum(1 for s in self._sats if s.used)
            fix = GnssFix(
                lat=lat,
                lon=lon,
                quality=int(self.sim_cfg.gps_quality),
                sats=used or tracked,
                hdop=max(0.5, 1.6 - 0.04 * used),  # rough HDOP from sat geometry
                alt_m=100.0,
                timestamp_mono=time.monotonic(),
            )
            with self._lock:
                self._latest = fix
                self._gga_count += 1

            # Publish at nominal cadence per SIM second — i.e. shrink the real
            # period when the world is sped up so the estimator sees a fix per
            # simulated second regardless of scale.
            scale = max(0.1, self.world.time_scale)
            wall_period = max(0.02, nominal_period / scale)
            next_tick += wall_period
            slack = next_tick - time.monotonic()
            if slack > 0:
                if self._stop.wait(slack):
                    return
            else:
                next_tick = time.monotonic()


class SimNtrip:
    """No-op NTRIP forwarder — sim doesn't need RTCM corrections."""

    enabled = False

    def start(self) -> None:
        return

    def stop(self) -> None:
        return


# ---- IMU + odometry ----------------------------------------------------

class SimIMU:
    """Stand-in for :class:`lawnbot.sensors.imu.IMU`."""

    def __init__(self, world: SimWorld, sim_cfg: SimCfg):
        self.world = world
        self.sim_cfg = sim_cfg

    def yaw_rad(self) -> float:
        _, _, th = self.world.true_pose()
        noise = float(self.sim_cfg.imu_noise_rad)
        if noise > 0:
            th += self.world.rng.gauss(0.0, noise)
        return math.atan2(math.sin(th), math.cos(th))

    @property
    def has_hardware(self) -> bool:
        return False


class SimOdometry:
    """Stand-in for :class:`lawnbot.sensors.odometry.QuadratureEncoder`."""

    def __init__(self, world: SimWorld):
        self.world = world

    def read_delta_m(self) -> float:
        return self.world.read_odom_delta()

    def close(self) -> None:
        return


# ---- PiSugar -----------------------------------------------------------

class SimPiSugar:
    """Stand-in for :class:`lawnbot.power.pisugar.PiSugar`."""

    def __init__(self, world: SimWorld):
        self.world = world
        self.state = BatteryState()

    def poll(self) -> BatteryState:
        self.state.percent = self.world.battery_percent()
        self.state.charging = False
        self.state.available = True
        self.state.last_read_mono = time.monotonic()
        return BatteryState(**self.state.__dict__)
