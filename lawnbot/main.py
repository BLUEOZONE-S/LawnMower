"""LawnBot main scheduler — wires every subsystem.

Threads:
  - Control loop @ ctrl_hz on SCHED_FIFO (real-time)
  - Safety monitor @ 10 Hz (its own thread, started by SafetyMonitor)
  - GPS reader @ ~UART rate (started by LC29H)
  - Asyncio loop on the main thread for the FastAPI/WebSocket UI

The Runtime object is the central hub. Subsystems take a handle to it.
The UI server gets `.snapshot()` for state and `.command(name, payload)`
for control.
"""
from __future__ import annotations

import logging
import math
import os
import signal
import sys
import threading
import time
from collections import deque
from pathlib import Path

from . import config
from .drive.kinematics import vd_to_command
from .estimator import Estimator
from .hardware import make_hardware
from .hardware_probe import HardwareProbe
from .nav.controller import Controller
from .nav.geo import Origin, centroid_ll, to_enu
from .nav.geometry import Polygon, bbox, point_in_polygon
from .nav.mission import Mission, State
from .nav.planner import PlanParams, plan_coverage
from .nav.teach import TeachRecorder, load_boundary_yaml
from .safety.monitor import SafetyMonitor
from .safety.stuck import StuckDetector
from .telemetry.logger import JsonlLogger
from .teleop import Teleop


log = logging.getLogger("lawnbot")


class Runtime:
    """Central hub. Owns hardware handles, sensor state, and mission state."""

    def __init__(self, cfg):
        self.cfg = cfg
        self.events: deque[str] = deque(maxlen=64)

        # ---- hardware (real or simulated) -------------------------------
        log.info("opening hardware backend")
        hw = make_hardware(cfg)
        self.motors = hw.motors
        self.servo = hw.servo
        self.gps = hw.gps
        self.imu = hw.imu
        self.odom = hw.odom
        self.pisugar = hw.pisugar
        self.ntrip = hw.ntrip
        self._sim_world = hw.world  # None outside sim mode
        self.pisugar.poll()

        # ---- world model ------------------------------------------------
        self.origin: Origin | None = None
        if cfg.sim.enabled:
            # In sim, the world's lat/lon anchor is authoritative — use it as
            # the estimator origin too so ENU coordinates stay coherent.
            self.origin = Origin(lat=cfg.sim.origin_lat, lon=cfg.sim.origin_lon)
        self.boundary_enu: Polygon | None = None
        self.keepouts_enu: list[Polygon] = []
        self.path: list[tuple[float, float]] = []
        self.covered: list[tuple[float, float]] = []
        self.last_gps_xy: tuple[float, float] | None = None

        # ---- algorithms -------------------------------------------------
        self.controller = Controller(cfg.control, cfg.geometry)
        self.mission = Mission(cfg.control)
        self.stuck = StuckDetector(cfg.stuck)
        self.teleop = Teleop(cfg.teleop, cfg.geometry)

        self.estimator: Estimator | None = None  # built once origin is known

        self.teach: TeachRecorder | None = None

        # ---- safety -----------------------------------------------------
        self.safety = SafetyMonitor(
            cfg.safety,
            min_fix_quality=cfg.gnss.min_fix_quality,
            estimator=None,  # set after we have the estimator
            pisugar=self.pisugar,
            boundary_provider=lambda: self.boundary_enu,
        )
        self.safety.on_trip = self._on_safety_trip

        # ---- logging ----------------------------------------------------
        log_dir = Path(os.environ.get("LAWNBOT_LOG_DIR", cfg.logging.dir))
        try:
            self.logger = JsonlLogger(log_dir / "telemetry.jsonl")
        except OSError as exc:
            log.warning("telemetry log dir %s unavailable (%s) — using ./logs", log_dir, exc)
            self.logger = JsonlLogger(Path("./logs/telemetry.jsonl"))

        # Try to load a previously taught boundary, if present.
        for candidate in self._boundary_candidates():
            try:
                self._load_boundary(candidate)
                break
            except FileNotFoundError:
                continue
        else:
            log.info("no boundary file found — start with teach mode")

        # Bootstrap the estimator once we have an origin.
        self._ensure_estimator()
        self.safety.est = self.estimator
        self.safety.start()

        # NTRIP forwarder
        try:
            self.ntrip.start()
        except Exception as e:
            log.warning("NTRIP not started: %s", e)

        # Live hardware/host probe — surfaced in snapshot() for the UI status panel.
        self.hw_probe = HardwareProbe(scan_interval_s=2.0)
        try:
            self.hw_probe.attach_gps_reader(self.gps)
        except Exception:
            pass

        # Plan-time defaults exposed in the snapshot so the UI sliders mirror
        # the current pattern (deck, overlap, headland, axis).
        default_headland = 0.0
        if cfg.platform == "ackermann":
            default_headland = cfg.geometry.min_turn_radius_m + 0.05
        self._last_plan_params = {
            "deck_m": cfg.geometry.deck_m,
            "body_clearance_m": cfg.geometry.body_clearance_m,
            "headland_m": default_headland,
            "overlap_pct": 0.0,
            "crosscut": True,
            "primary_axis": "h",
            "pattern": "boustrophedon",
        }

        # PID auto-tuner — built lazily; the UI flips it on/off.
        from .nav.autotune import AutoTuner
        self.autotuner = AutoTuner(self.controller, self.cfg.control)

        self._stop = threading.Event()
        self._ctrl_thread = threading.Thread(target=self._control_loop, daemon=True)
        self._last_odom_ds = 0.0
        # Accumulated simulated-time elapsed by the control loop, used to keep
        # the stuck detector calibrated when the sim runs faster than wall time.
        self._sim_time_elapsed = 0.0

    # ---- public API for the UI -----------------------------------------

    def snapshot(self) -> dict:
        pose = self.estimator.snapshot() if self.estimator else None
        gps = self.gps.latest
        mstat = self.mission.snapshot()
        sstat = self.safety.snapshot()

        # Per-satellite data for the GNSS debug panel. Real LC29H and SimLC29H
        # both expose this; older mocks without it just return an empty list.
        sat_payload: list[dict] = []
        sat_age = None
        try:
            ssnap = self.gps.satellites
        except AttributeError:
            ssnap = None
        if ssnap is not None and getattr(ssnap, "sats", None):
            sat_age = ssnap.age_s
            for s in ssnap.sats:
                sat_payload.append({
                    "prn": s.prn,
                    "talker": s.talker,
                    "constellation": s.constellation,
                    "el": s.elevation_deg,
                    "az": s.azimuth_deg,
                    "snr": s.snr_dbhz,
                    "used": bool(s.used),
                })

        out = {
            "pose": {"x": pose.x, "y": pose.y, "theta": pose.theta} if pose else None,
            "gps": {
                "quality": gps.quality if gps else 0,
                "sats": gps.sats if gps else 0,
                "hdop": gps.hdop if gps else 0,
                "age_s": gps.age_s if gps else None,
                "lat": gps.lat if gps else None,
                "lon": gps.lon if gps else None,
                "alt_m": gps.alt_m if gps else None,
                "satellites": sat_payload,
                "sats_age_s": sat_age,
            },
            "gps_xy": self.last_gps_xy,
            "battery": self._battery_snapshot(sstat),
            "mission": {
                "state": mstat.state.value if hasattr(mstat.state, "value") else str(mstat.state),
                "waypoint_idx": mstat.waypoint_idx,
                "n_waypoints": mstat.n_waypoints,
                "coverage_pct": mstat.coverage_pct,
                "distance_m": mstat.distance_m,
                "note": mstat.note,
            },
            "control": {
                "heading_err": getattr(self, "_last_ctl_err", 0.0),
                "cross_track": getattr(self, "_last_ctl_cross", 0.0),
                "pid": self.controller.pid.breakdown,
            },
            "target": getattr(self, "_last_target", None),
            "reach": self.cfg.control.reach_m,
            "pattern": self._last_plan_params,
            "sim": {
                "enabled": self.cfg.sim.enabled,
                "time_scale": self._sim_world.time_scale if self._sim_world else 1.0,
            },
            "backend": "sim" if self._sim_world is not None else "real",
            "autotune": self.autotuner.snapshot(),
            "boundary": list(self.boundary_enu) if self.boundary_enu else None,
            "keepouts": [{"label": "k", "points": list(p)} for p in self.keepouts_enu],
            "path": list(self.path),
            "covered": list(self.covered),
            "teach": self.teach.snapshot() if self.teach else None,
            "events": list(self.events),
        }
        # Merge in hardware-presence + host metrics for the UI status panel.
        try:
            out.update(self.hw_probe.snapshot())
        except Exception:
            log.exception("hw_probe.snapshot failed")
        return out

    def _battery_snapshot(self, sstat) -> dict:
        """Rich PiSugar battery state for the UI. Falls back to the safety-monitor
        view if the PiSugar socket is down or the driver doesn't expose details."""
        st = getattr(self.pisugar, "state", None)
        if st is not None and getattr(st, "available", False):
            return {
                "percent": st.percent,
                "charging": st.charging,
                "plugged": getattr(st, "plugged", False),
                "voltage_v": getattr(st, "voltage_v", 0.0),
                "current_a": getattr(st, "current_a", 0.0),
                "model": getattr(st, "model", ""),
                "available": True,
            }
        return {
            "percent": sstat.battery_pct,
            "charging": sstat.charging,
            "plugged": False,
            "voltage_v": 0.0,
            "current_a": 0.0,
            "model": "",
            "available": False,
        }

    def command(self, name: str, payload: dict) -> dict:
        handlers = {
            "mission.plan": self._cmd_plan,
            "mission.start": self._cmd_mission_start,
            "mission.pause": lambda _: (self.mission.pause(), self.event("mission pause"))[1] or {"ok": True},
            "mission.resume": self._cmd_mission_resume,
            "mission.stop": lambda _: (self.mission.stop(), self.motors.stop(), self.event("mission stop"))[2] or {"ok": True},
            "mission.replan": self._cmd_replan,
            "mode.toggle": self._cmd_mode_toggle,
            "teach.perimeter": lambda _: (self._ensure_teach(), self.teach.start_perimeter(), self.event("teach perimeter"))[2] or {"ok": True},
            "teach.keepout": lambda p: (self._ensure_teach(), self.teach.start_keepout(p.get("label", "")), self.event("teach keep-out"))[2] or {"ok": True},
            "teach.finish": self._cmd_teach_finish,
            "teach.undo": lambda _: (self.teach and self.teach.undo(), {"ok": True})[1],
            "teach.save": self._cmd_teach_save,
            "control.tune": self._cmd_tune,
            "teleop": self._cmd_teleop,
            "sim.reset": self._cmd_sim_reset,
            "sim.reset_run": self._cmd_sim_reset_run,
            "sim.speed": self._cmd_sim_speed,
            "autotune.start": self._cmd_autotune_start,
            "autotune.stop": self._cmd_autotune_stop,
            "backend.swap": self._cmd_backend_swap,
        }
        h = handlers.get(name)
        if not h:
            return {"error": f"unknown cmd {name}"}
        try:
            return h(payload) or {"ok": True}
        except Exception as e:
            log.exception("cmd %s failed", name)
            return {"error": str(e)}

    def event(self, msg: str) -> None:
        ts = time.strftime("%H:%M:%S")
        self.events.append(f"{ts} {msg}")
        log.info(msg)

    # ---- command handlers ---------------------------------------------

    def _cmd_plan(self, payload):
        if not self.boundary_enu:
            return {"error": "no boundary — teach one first"}
        # Per-platform default headland (R_min + 5cm for Ackermann; 0 for diff).
        default_headland = 0.0
        if self.cfg.platform == "ackermann":
            default_headland = self.cfg.geometry.min_turn_radius_m + 0.05

        p = payload if isinstance(payload, dict) else {}

        def _f(key, default):
            try:
                return float(p[key]) if key in p else default
            except (TypeError, ValueError):
                return default

        deck_m = max(0.1, _f("deck_m", self.cfg.geometry.deck_m))
        body_clearance = max(0.0, _f("body_clearance_m", self.cfg.geometry.body_clearance_m))
        headland = max(0.0, _f("headland_m", default_headland))
        overlap = max(0.0, min(0.5, _f("overlap_pct", 0.0)))
        crosscut = bool(p.get("crosscut", True))
        primary_axis = p.get("primary_axis", "h")
        if primary_axis not in ("h", "v"):
            primary_axis = "h"
        from .nav.planner import PATTERN_NAMES
        pattern = str(p.get("pattern", "boustrophedon")).lower()
        if pattern not in PATTERN_NAMES:
            pattern = "boustrophedon"

        params = PlanParams(
            deck_m=deck_m,
            body_clearance_m=body_clearance,
            keepout_inflate_m=body_clearance,
            crosscut=crosscut,
            headland_m=headland,
            overlap_pct=overlap,
            primary_axis=primary_axis,
            pattern=pattern,
        )
        self.path = plan_coverage(self.boundary_enu, self.keepouts_enu, params)
        self.mission.load_path(self.path)
        # Remember params so the UI sliders can mirror them.
        self._last_plan_params = {
            "deck_m": deck_m,
            "body_clearance_m": body_clearance,
            "headland_m": headland,
            "overlap_pct": overlap,
            "crosscut": crosscut,
            "primary_axis": primary_axis,
            "pattern": pattern,
        }
        self.event(
            f"plan: {len(self.path)} waypoints — {pattern} "
            f"(deck={deck_m:.2f}m overlap={overlap*100:.0f}% headland={headland:.2f}m)"
        )
        return {"ok": True, "n": len(self.path), "params": self._last_plan_params}

    def _cmd_replan(self, _payload):
        return self._cmd_plan(_payload)

    def _prep_for_auto(self) -> None:
        """Common reset path before re-entering AUTO from PAUSED/MANUAL/STUCK.

        Clears the stuck detector's recovery counter (otherwise the first blip
        after restart re-trips STUCK because tries are already exhausted),
        re-arms safety (a previous geofence breach would otherwise keep the
        rover disarmed), resets the PID integrator so the I-term from the
        prior run doesn't slam the steer at start-up, and refreshes the
        mission's cached pose so the nearest-waypoint snap uses where the
        operator actually drove the rover, not where it got stuck.
        """
        self.stuck.reset()
        self.safety.rearm()
        self.controller.pid.reset()
        if self.estimator is not None:
            pose = self.estimator.snapshot()
            self.mission.update_pose(pose)

    def _cmd_mission_start(self, _payload):
        if not self.mission.snapshot().n_waypoints:
            return {"error": "no path — click Plan first"}
        self._prep_for_auto()
        self.mission.start()
        self.event("mission start")
        return {"ok": True}

    def _cmd_mission_resume(self, _payload):
        prev = self.mission.snapshot().state
        self._prep_for_auto()
        self.mission.resume()
        new = self.mission.snapshot().state
        if new != State.AUTO:
            return {"error": f"resume from {prev.value} not possible"}
        self.event(f"resume → AUTO (from {prev.value})")
        return {"ok": True}

    def _cmd_mode_toggle(self, _payload):
        st = self.mission.snapshot().state
        if st == State.MANUAL:
            self._prep_for_auto()
            self.mission.resume()
            self.event("AUTO")
        else:
            self.mission.to_manual()
            self.motors.stop()
            self.event("MANUAL")
        return {"ok": True}

    def _cmd_teach_finish(self, _payload):
        if not self.teach:
            return {"error": "not teaching"}
        msg = self.teach.finish()
        self.event(f"teach: {msg}")
        # Sync world model with whatever was just finalized.
        if self.teach.boundary:
            self.boundary_enu = self.teach.boundary
        self.keepouts_enu = [poly for _label, poly in self.teach.keepouts]
        return {"ok": True, "msg": msg}

    def _cmd_teach_save(self, _payload):
        if not self.teach:
            return {"error": "no teach session"}
        if not self.teach.boundary or len(self.teach.boundary) < 3:
            return {"error": "nothing to save — record a perimeter first"}
        self.teach.save_yaml("boundary.yaml")
        self.event("boundary.yaml saved")
        return {"ok": True}

    def _cmd_tune(self, payload):
        self.controller.update_gains(**{k: payload[k] for k in (
            "kp", "ki", "kd", "v_nominal", "lookahead_m") if k in payload})
        return {"ok": True}

    def _cmd_teleop(self, payload):
        v = float(payload.get("v", 0.0)) * self.cfg.teleop.manual_v_max
        delta = float(payload.get("delta", 0.0)) * self.cfg.geometry.steer_max_rad
        self.teleop.ingest(v, delta, override_geofence=bool(payload.get("override", False)))
        return {"ok": True}

    def _random_in_cut_zone(self, pad: float = 0.6) -> tuple[float, float] | None:
        """Sample a random ENU point that's inside the boundary, outside every
        keep-out, and at least ``pad`` meters from the boundary's bounding box
        edge so the rover doesn't spawn right against the geofence.
        """
        import random
        if not self.boundary_enu:
            return None
        x0, y0, x1, y1 = bbox(self.boundary_enu)
        if (x1 - x0) < 2 * pad or (y1 - y0) < 2 * pad:
            pad = min(0.1, 0.4 * min(x1 - x0, y1 - y0))
        rng = random.Random()
        for _ in range(400):
            px = rng.uniform(x0 + pad, x1 - pad)
            py = rng.uniform(y0 + pad, y1 - pad)
            if not point_in_polygon((px, py), self.boundary_enu):
                continue
            # Add a small clearance buffer around keep-outs.
            blocked = False
            for ko in self.keepouts_enu:
                if point_in_polygon((px, py), ko):
                    blocked = True; break
            if blocked:
                continue
            return (px, py)
        return None

    def _cmd_sim_reset(self, payload):
        """Teleport the sim rover to a random in-bounds, outside-keepout pose
        and reset the mission/estimator/safety to a clean IDLE state.
        """
        if not self.cfg.sim.enabled or self._sim_world is None:
            return {"error": "sim.reset only available in sim mode"}
        if not self.boundary_enu:
            return {"error": "no boundary loaded — nothing to spawn into"}

        spawn = self._random_in_cut_zone()
        if spawn is None:
            return {"error": "could not find an in-bounds spawn point"}
        x, y = spawn
        theta = float(payload.get("theta_rad", 0.0)) if isinstance(payload, dict) and "theta_rad" in payload else 0.0
        if not (isinstance(payload, dict) and "theta_rad" in payload):
            import random
            theta = random.Random().uniform(-math.pi, math.pi)

        # 1) Stop actuators before teleporting so nothing surprising fires.
        self.motors.stop()
        self.servo.center()

        # 2) Teleport ground truth.
        self._sim_world.set_pose(x, y, theta)

        # 3) Reset everything that holds path / pose state.
        self.path = []
        self.covered.clear()
        self.mission.load_path([])
        self.mission.stop()             # → IDLE
        self.stuck.reset()
        self.controller.pid.reset()
        self.teleop.ingest(0.0, 0.0)
        if self.estimator is not None:
            self.estimator.seed(x, y, theta)
        self.last_gps_xy = (x, y)
        for attr in ("_last_target", "_last_ctl_err", "_last_ctl_cross"):
            if hasattr(self, attr):
                delattr(self, attr)

        # 4) Re-arm safety (geofence breach state from a prior run is cleared).
        self.safety.rearm()

        self.event(f"sim reset → ({x:.2f}, {y:.2f}, θ={math.degrees(theta):.0f}°)")
        return {"ok": True, "x": x, "y": y, "theta_rad": theta}

    def _cmd_backend_swap(self, payload):
        """Hot-swap between sim and real Pi hardware *in this process*.

        Only meaningful on the Pi (Windows can't open I2C/pigpio/UART). On a
        machine that can't open real hardware, the rebuild fails and we
        gracefully roll back to the prior backend so the rover doesn't end up
        with a half-initialized HAT.
        """
        target = (payload or {}).get("target", "").lower()
        if target not in ("sim", "real"):
            return {"error": "target must be 'sim' or 'real'"}
        currently_sim = self._sim_world is not None
        want_sim = (target == "sim")
        if currently_sim == want_sim:
            return {"ok": True, "msg": f"already on {target} backend"}
        return self.swap_backend(want_sim)

    def swap_backend(self, want_sim: bool) -> dict:
        log.info("swap_backend → %s", "sim" if want_sim else "real")
        # 1) Pause the control loop's hardware-touching dispatch via the safety
        # gate — the loop's "armed=False" branch just calls motors.stop() and
        # skips dispatch. We rearm at the end (or on rollback).
        was_armed = self.safety.snapshot().armed
        self.safety.request_stop("backend swap")

        # 2) Hold references to the old hardware so we can either tear them
        # down AFTER the new ones come up, or roll back if the build fails.
        old = {
            "motors": self.motors, "servo": self.servo, "gps": self.gps,
            "imu": self.imu, "odom": self.odom, "pisugar": self.pisugar,
            "ntrip": self.ntrip, "world": self._sim_world,
        }
        # Stop the wheels but leave the rest of the old backend alive in case
        # we need to roll back.
        try: old["motors"].stop()
        except Exception: pass

        # 3) Build new hardware. On Windows, target='real' will raise here
        # (smbus2/pigpio/etc. can't open hardware) → roll back cleanly.
        try:
            new_hw = make_hardware(self.cfg, force_sim=want_sim)
        except Exception as e:
            log.exception("swap_backend: build failed, rolling back")
            if was_armed:
                self.safety.rearm()
            return {"error": f"failed to build {('sim' if want_sim else 'real')} backend: {e}"}

        # 4) Wire new hardware in.
        self.motors = new_hw.motors
        self.servo = new_hw.servo
        self.gps = new_hw.gps
        self.imu = new_hw.imu
        self.odom = new_hw.odom
        self.pisugar = new_hw.pisugar
        self.ntrip = new_hw.ntrip
        self._sim_world = new_hw.world
        try: self.ntrip.start()
        except Exception as e: log.warning("ntrip start after swap: %s", e)

        # 5) Tear down the old hardware in the background so we don't block on
        # a slow socket close. NTRIP must stop here, not before, so a failed
        # build leaves the old corrections stream running.
        def _close_old():
            try: old["ntrip"].stop()
            except Exception: pass
            for k in ("motors", "servo", "gps"):
                try: getattr(old[k], "close")()
                except Exception: pass
            if old.get("world"):
                try: old["world"].stop()
                except Exception: pass
        threading.Thread(target=_close_old, daemon=True).start()

        # 6) Reset world model state — new backend has its own pose.
        self.path = []
        self.covered.clear()
        self.mission.load_path([])
        self.mission.stop()
        self.stuck.reset()
        self.controller.pid.reset()
        self._last_target = None

        # 7) Seed estimator + safety appropriately for the new backend.
        self.safety.pisugar = self.pisugar
        if want_sim:
            s = self.cfg.sim
            self.origin = Origin(lat=s.origin_lat, lon=s.origin_lon)
            self.estimator = Estimator(self.cfg.estimator, self.origin)
            self.estimator.seed(s.start_x, s.start_y, math.radians(s.start_theta_deg))
            self.safety.est = self.estimator
        else:
            # Real backend: wait for the first GPS fix to anchor.
            self.origin = None
            self.estimator = None
            self._ensure_estimator()
            self.safety.est = self.estimator

        self.safety.rearm()
        self.event(f"backend swapped → {'SIM' if want_sim else 'REAL'}")
        return {"ok": True, "backend": "sim" if want_sim else "real"}

    def _cmd_autotune_start(self, _payload):
        st = self.mission.snapshot().state
        if st not in (State.AUTO, State.RECOVER):
            return {"error": "auto-tune needs the mission running (AUTO state)"}
        if not self.autotuner.start():
            return {"error": "auto-tune already running"}
        self.event("auto-tune started")
        return {"ok": True}

    def _cmd_autotune_stop(self, _payload):
        if not self.autotuner.status.running:
            return {"error": "auto-tune is not running"}
        self.autotuner.stop()
        self.event("auto-tune stopped")
        return {"ok": True}

    def _cmd_sim_speed(self, payload):
        if self._sim_world is None:
            return {"error": "sim mode only"}
        try:
            scale = float(payload.get("scale", 1.0))
        except (TypeError, ValueError):
            return {"error": "scale must be a number"}
        self._sim_world.set_time_scale(scale)
        self.event(f"sim speed × {self._sim_world.time_scale:.2f}")
        return {"ok": True, "scale": self._sim_world.time_scale}

    def _cmd_sim_reset_run(self, payload):
        """One-shot: reset, plan, start. Convenient demo button."""
        result = self._cmd_sim_reset(payload)
        if "error" in result:
            return result
        # Brief settle so the estimator's first dead-reckon sees the new pose.
        time.sleep(0.05)
        plan = self._cmd_plan({})
        if "error" in plan:
            return {"reset": result, "plan_error": plan["error"]}
        self.mission.start()
        self.event("mission start (after reset)")
        return {"ok": True, "reset": result, "n_waypoints": plan.get("n", 0)}

    # ---- internals -----------------------------------------------------

    def _boundary_candidates(self) -> list[str]:
        """boundary.yaml wins; in sim mode also fall back to boundary.sim.yaml."""
        candidates = ["boundary.yaml"]
        if self.cfg.sim.enabled:
            candidates.append("boundary.sim.yaml")
        return candidates

    def _load_boundary(self, path: str | Path) -> None:
        doc = load_boundary_yaml(path)
        b_ll = [(p["lat"], p["lon"]) for p in doc.get("boundary", [])]
        # A 0/1/2-point "polygon" is not a valid geofence — treat the file as
        # absent so the fallback chain (boundary.sim.yaml, then teach mode) runs.
        if len(b_ll) < 3:
            log.warning("boundary file %s has only %d points — skipping", path, len(b_ll))
            raise FileNotFoundError(path)
        o = doc["origin"]
        self.origin = Origin(lat=o["lat"], lon=o["lon"])
        self.boundary_enu = [to_enu(lat, lon, self.origin) for lat, lon in b_ll]
        self.keepouts_enu = []
        for k in doc.get("keepouts", []):
            if k.get("type") == "polygon":
                pts = [(p["lat"], p["lon"]) for p in k["points"]]
                if len(pts) < 3:
                    continue
                self.keepouts_enu.append([to_enu(lat, lon, self.origin) for lat, lon in pts])
        self.event(f"boundary loaded from {path}: {len(self.boundary_enu)} pts, {len(self.keepouts_enu)} keep-outs")

    def _ensure_estimator(self) -> None:
        if self.estimator is not None:
            return
        if self.origin is None:
            # No taught boundary yet — defer origin until the first GPS fix.
            fix = self.gps.latest
            if fix is None:
                # Bootstrap with placeholder; gets overwritten on first fix.
                self.origin = Origin(lat=0.0, lon=0.0)
            else:
                self.origin = Origin(lat=fix.lat, lon=fix.lon)
        self.estimator = Estimator(self.cfg.estimator, self.origin)
        # In sim, the world starts at a known pose (cfg.sim.start). Seed the
        # estimator there so dead-reckoning starts from the right place — else
        # the controller would chase a 1-second pose-snap from (0,0) and might
        # spin the rover off course before the first GPS fix lands.
        if self.cfg.sim.enabled:
            s = self.cfg.sim
            self.estimator.seed(s.start_x, s.start_y, math.radians(s.start_theta_deg))

    def _ensure_teach(self) -> None:
        if self.teach is None:
            self._ensure_estimator()
            assert self.origin is not None
            self.teach = TeachRecorder(self.cfg.teach, self.origin)

    def _on_safety_trip(self, reason: str) -> None:
        self.motors.stop()
        self.event(f"SAFETY TRIP: {reason}")

    def _control_loop(self) -> None:
        # Attempt SCHED_FIFO for deterministic timing. Will silently fall back
        # to default scheduling if the process lacks CAP_SYS_NICE.
        try:
            os.sched_setscheduler(0, os.SCHED_FIFO, os.sched_param(40))
            log.info("control thread: SCHED_FIFO acquired")
        except (OSError, AttributeError) as e:
            log.warning("SCHED_FIFO unavailable: %s", e)

        period = 1.0 / self.cfg.control.ctrl_hz
        last = time.monotonic()
        while not self._stop.is_set():
            now = time.monotonic()
            dt_real = max(1e-3, now - last)
            last = now
            # In sim, the world advances `time_scale × dt_real` of simulated
            # time between ticks. Pass SIM dt to the controller so PID gains
            # stay calibrated regardless of the speed slider (otherwise the
            # D-term sees scale× larger heading-error rate and the controller
            # oscillates at high speed).
            scale = self._sim_world.time_scale if self._sim_world else 1.0
            dt = dt_real * scale
            self._sim_time_elapsed += dt
            self.safety.pet()

            # 1) Pull sensors.
            yaw = self.imu.yaw_rad()
            ds = self.odom.read_delta_m()
            pose = self.estimator.dead_reckon(yaw, ds) if self.estimator else None

            # 2) Ingest fresh GPS.
            fix = self.gps.latest
            if fix is not None and self.estimator is not None:
                if self.origin is not None and self.origin.lat == 0.0:
                    self.origin = Origin(lat=fix.lat, lon=fix.lon)
                    self.estimator = Estimator(self.cfg.estimator, self.origin)
                    self.safety.est = self.estimator
                    pose = self.estimator.snapshot()
                if fix.age_s < 1.5 and pose is not None:
                    self.estimator.ingest_gps(fix)
                from .nav.geo import to_enu as _enu
                self.last_gps_xy = _enu(fix.lat, fix.lon, self.origin)

            # Keep the mission's cached pose current even when not in AUTO,
            # so a MANUAL→AUTO toggle (or STUCK→Start) can snap to the nearest
            # not-yet-covered waypoint from where the operator actually moved
            # the rover, not from where it was when we left AUTO.
            if pose is not None:
                self.mission.update_pose(pose)

            # 3) Dispatch by mission state.
            sstate = self.mission.snapshot().state
            armed = self.safety.snapshot().armed
            if not armed:
                self.motors.stop()
                self.servo.center()
            elif sstate == State.MANUAL:
                cmd = self.teleop.command_for_motors(self.cfg.drive)
                if cmd is None:
                    self.motors.stop()
                else:
                    duty, steer = cmd
                    if self.cfg.teleop.enforce_geofence and self.boundary_enu and pose is not None:
                        if not point_in_polygon((pose.x, pose.y), self.boundary_enu):
                            self.motors.stop()
                            continue
                    self.motors.set_throttle(duty)
                    self.servo.set_steer(steer)
            elif sstate in (State.AUTO, State.RECOVER) and pose is not None:
                target, done = self.mission.update(pose)
                if done:
                    self.motors.stop()
                    self.event("mission done")
                elif target is not None:
                    self._last_target = target
                    out = self.controller.step(pose, self.path[self.mission.snapshot().waypoint_idx:], dt)
                    self._last_ctl_err = out.heading_err
                    self._last_ctl_cross = out.cross_track
                    self.autotuner.sample(out.heading_err, out.cross_track)
                    cmd = vd_to_command(out.v, out.delta, self.cfg.drive, self.cfg.geometry)
                    self.motors.set_throttle(cmd.throttle_duty)
                    self.servo.set_steer(cmd.steer_rad)
                    # Track covered cells coarsely.
                    if not self.covered or math.hypot(pose.x - self.covered[-1][0], pose.y - self.covered[-1][1]) > 0.2:
                        self.covered.append((pose.x, pose.y))
                    # Stuck detection.
                    outcome = self.stuck.update(
                        pose,
                        commanding_motion=cmd.throttle_duty > 0.02,
                        odom_delta_m=ds,
                        now=self._sim_time_elapsed,
                    )
                    if outcome.state == "stuck":
                        self.mission.to_stuck(outcome.reason)
                        self.motors.stop()
                        self.event(f"STUCK: {outcome.reason}")
                    elif outcome.state == "recover":
                        self.mission.to_recover()
                        self.event(f"RECOVER: {outcome.reason}")
            else:
                self.motors.stop()

            # 4) Teach breadcrumb sampling.
            if self.teach and pose is not None:
                self.teach.sample((pose.x, pose.y))

            # 5) Telemetry.
            self.logger.log({
                "x": pose.x if pose else None,
                "y": pose.y if pose else None,
                "th": pose.theta if pose else None,
                "fq": fix.quality if fix else None,
                "state": sstate.value if hasattr(sstate, "value") else str(sstate),
            })

            # 6) Pace.
            slack = period - (time.monotonic() - now)
            if slack > 0:
                time.sleep(slack)

    def run_forever(self) -> None:
        self._ctrl_thread.start()
        log.info("control thread started")

    def shutdown(self) -> None:
        self._stop.set()
        try:
            self.autotuner.stop()
        except Exception:
            log.exception("autotuner.stop failed")
        try:
            self.ntrip.stop()
        except Exception:
            log.exception("ntrip.stop failed")
        self.safety.stop()
        try:
            self.motors.close()
        finally:
            try:
                self.servo.close()
            finally:
                try:
                    self.gps.close()
                finally:
                    if self._sim_world is not None:
                        try:
                            self._sim_world.stop()
                        except Exception:
                            log.exception("sim world stop failed")


# ---- entrypoint ---------------------------------------------------------

def main(argv: list[str] | None = None) -> int:
    import argparse

    logging.basicConfig(level=logging.INFO,
                        format="%(asctime)s %(levelname)s %(name)s: %(message)s")

    parser = argparse.ArgumentParser(prog="lawnbot", description="LawnBot scheduler")
    parser.add_argument(
        "--config", "-c",
        default=os.environ.get("LAWNBOT_CONFIG", "config.yaml"),
        help="path to config.yaml (default: config.yaml or $LAWNBOT_CONFIG)",
    )
    parser.add_argument(
        "--sim", action="store_true",
        help="shortcut for --config config.sim.yaml",
    )
    args = parser.parse_args(argv)

    cfg_path = "config.sim.yaml" if args.sim else args.config
    cfg = config.load(cfg_path)
    log.info("config loaded from %s — sim=%s ctrl_hz=%d ui_port=%d",
             cfg_path, cfg.sim.enabled, cfg.control.ctrl_hz, cfg.ui.port)

    runtime = Runtime(cfg)
    runtime.run_forever()

    stop_evt = threading.Event()
    # SIGTERM is available on POSIX; on Windows signal.signal accepts it but
    # only SIGINT and SIGBREAK actually fire. Register defensively.
    for sig_name in ("SIGTERM", "SIGINT", "SIGBREAK"):
        sig = getattr(signal, sig_name, None)
        if sig is None:
            continue
        try:
            signal.signal(sig, lambda *_: stop_evt.set())
        except (ValueError, OSError):
            pass  # not supported on this platform

    # Start the FastAPI server in the main asyncio loop.
    import asyncio
    import uvicorn

    from .ui.server import build_app

    # Windows has no uvloop — let uvicorn pick the best available loop.
    loop_choice = "auto" if sys.platform != "win32" else "asyncio"

    app = build_app(runtime, push_hz=cfg.ui.push_hz)
    server_cfg = uvicorn.Config(
        app, host="0.0.0.0", port=cfg.ui.port, log_level="info",
        loop=loop_choice, workers=1,
    )
    server = uvicorn.Server(server_cfg)

    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    server_task = loop.create_task(server.serve())

    try:
        loop.run_until_complete(_wait_for_stop(stop_evt, server))
    finally:
        server_task.cancel()
        runtime.shutdown()
        loop.close()
    return 0


async def _wait_for_stop(stop_evt: threading.Event, server) -> None:
    while not stop_evt.is_set():
        import asyncio
        await asyncio.sleep(0.25)
    server.should_exit = True


if __name__ == "__main__":
    sys.exit(main())
