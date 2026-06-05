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
from .drive.motor_hat import MotorHAT
from .drive.servo import SteeringServo
from .estimator import Estimator
from .gnss.lc29h import LC29H
from .gnss.ntrip import NtripForwarder
from .nav.controller import Controller
from .nav.geo import Origin, centroid_ll, to_enu
from .nav.geometry import Polygon, point_in_polygon
from .nav.mission import Mission, State
from .nav.planner import PlanParams, plan_coverage
from .nav.teach import TeachRecorder, load_boundary_yaml
from .power.pisugar import PiSugar
from .safety.monitor import SafetyMonitor
from .safety.stuck import StuckDetector
from .sensors.imu import IMU, StubIMU
from .sensors.odometry import StubOdometry
from .telemetry.logger import JsonlLogger
from .teleop import Teleop


log = logging.getLogger("lawnbot")


class Runtime:
    """Central hub. Owns hardware handles, sensor state, and mission state."""

    def __init__(self, cfg):
        self.cfg = cfg
        self.events: deque[str] = deque(maxlen=64)

        # ---- hardware ---------------------------------------------------
        log.info("opening motor HAT + servo")
        self.motors = MotorHAT(cfg.drive, timeout_ms=cfg.safety.motor_timeout_ms)
        self.servo = SteeringServo(cfg.steering, cfg.geometry)
        self.servo.center()

        log.info("opening GPS UART %s @ %d", cfg.gnss.port, cfg.gnss.baud)
        self.gps = LC29H(cfg.gnss)

        log.info("opening IMU")
        try:
            self.imu = IMU()
        except Exception as e:
            log.warning("IMU unavailable (%s) — using StubIMU", e)
            self.imu = StubIMU()

        self.odom = StubOdometry()  # swap to QuadratureEncoder once wired

        self.pisugar = PiSugar()
        self.pisugar.poll()

        # ---- world model ------------------------------------------------
        self.origin: Origin | None = None
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
        self.logger = JsonlLogger(Path("/var/log/lawnbot/telemetry.jsonl"))

        # Try to load a previously taught boundary, if present.
        try:
            self._load_boundary("boundary.yaml")
        except FileNotFoundError:
            log.info("no boundary.yaml — start with teach mode")

        # Bootstrap the estimator once we have an origin.
        self._ensure_estimator()
        self.safety.est = self.estimator
        self.safety.start()

        # NTRIP forwarder
        self.ntrip = NtripForwarder(cfg.ntrip, self.gps.write_rtcm)
        try:
            self.ntrip.start()
        except Exception as e:
            log.warning("NTRIP not started: %s", e)

        self._stop = threading.Event()
        self._ctrl_thread = threading.Thread(target=self._control_loop, daemon=True)
        self._last_odom_ds = 0.0

    # ---- public API for the UI -----------------------------------------

    def snapshot(self) -> dict:
        pose = self.estimator.snapshot() if self.estimator else None
        gps = self.gps.latest
        mstat = self.mission.snapshot()
        sstat = self.safety.snapshot()
        out = {
            "pose": {"x": pose.x, "y": pose.y, "theta": pose.theta} if pose else None,
            "gps": {
                "quality": gps.quality if gps else 0,
                "sats": gps.sats if gps else 0,
                "hdop": gps.hdop if gps else 0,
                "age_s": gps.age_s if gps else None,
            },
            "gps_xy": self.last_gps_xy,
            "battery": {"percent": sstat.battery_pct, "charging": sstat.charging},
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
            "boundary": list(self.boundary_enu) if self.boundary_enu else None,
            "keepouts": [{"label": "k", "points": list(p)} for p in self.keepouts_enu],
            "path": list(self.path),
            "covered": list(self.covered),
            "teach": self.teach.snapshot() if self.teach else None,
            "events": list(self.events),
        }
        return out

    def command(self, name: str, payload: dict) -> dict:
        handlers = {
            "mission.plan": self._cmd_plan,
            "mission.start": lambda _: (self.mission.start(), self.event("mission start"))[1] or {"ok": True},
            "mission.pause": lambda _: (self.mission.pause(), self.event("mission pause"))[1] or {"ok": True},
            "mission.resume": lambda _: (self.mission.resume(), self.safety.rearm(), self.event("mission resume"))[2] or {"ok": True},
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

    def _cmd_plan(self, _payload):
        if not self.boundary_enu:
            return {"error": "no boundary — teach one first"}
        params = PlanParams(
            deck_m=self.cfg.geometry.deck_m,
            body_clearance_m=self.cfg.geometry.body_clearance_m,
            keepout_inflate_m=self.cfg.geometry.body_clearance_m,
            crosscut=True,
        )
        self.path = plan_coverage(self.boundary_enu, self.keepouts_enu, params)
        self.mission.load_path(self.path)
        self.event(f"plan: {len(self.path)} waypoints")
        return {"ok": True, "n": len(self.path)}

    def _cmd_replan(self, _payload):
        return self._cmd_plan(_payload)

    def _cmd_mode_toggle(self, _payload):
        st = self.mission.snapshot().state
        if st == State.MANUAL:
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

    # ---- internals -----------------------------------------------------

    def _load_boundary(self, path: str | Path) -> None:
        doc = load_boundary_yaml(path)
        o = doc["origin"]
        self.origin = Origin(lat=o["lat"], lon=o["lon"])
        b_ll = [(p["lat"], p["lon"]) for p in doc.get("boundary", [])]
        self.boundary_enu = [to_enu(lat, lon, self.origin) for lat, lon in b_ll]
        self.keepouts_enu = []
        for k in doc.get("keepouts", []):
            if k.get("type") == "polygon":
                pts = [(p["lat"], p["lon"]) for p in k["points"]]
                self.keepouts_enu.append([to_enu(lat, lon, self.origin) for lat, lon in pts])
        self.event(f"boundary loaded: {len(self.boundary_enu)} pts, {len(self.keepouts_enu)} keep-outs")

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
            dt = max(1e-3, now - last)
            last = now
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
                    cmd = vd_to_command(out.v, out.delta, self.cfg.drive, self.cfg.geometry)
                    self.motors.set_throttle(cmd.throttle_duty)
                    self.servo.set_steer(cmd.steer_rad)
                    # Track covered cells coarsely.
                    if not self.covered or math.hypot(pose.x - self.covered[-1][0], pose.y - self.covered[-1][1]) > 0.2:
                        self.covered.append((pose.x, pose.y))
                    # Stuck detection.
                    outcome = self.stuck.update(pose, commanding_motion=cmd.throttle_duty > 0.02, odom_delta_m=ds)
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
        self.ntrip.stop()
        self.safety.stop()
        try:
            self.motors.close()
        finally:
            try:
                self.servo.close()
            finally:
                self.gps.close()


# ---- entrypoint ---------------------------------------------------------

def main() -> int:
    logging.basicConfig(level=logging.INFO,
                        format="%(asctime)s %(levelname)s %(name)s: %(message)s")
    cfg = config.load()
    log.info("config loaded — ctrl_hz=%d ui_port=%d", cfg.control.ctrl_hz, cfg.ui.port)

    runtime = Runtime(cfg)
    runtime.run_forever()

    stop_evt = threading.Event()
    signal.signal(signal.SIGTERM, lambda *_: stop_evt.set())
    signal.signal(signal.SIGINT, lambda *_: stop_evt.set())

    # Start the FastAPI server in the main asyncio loop.
    import uvicorn

    from .ui.server import build_app
    app = build_app(runtime, push_hz=cfg.ui.push_hz)
    server_cfg = uvicorn.Config(app, host="0.0.0.0", port=cfg.ui.port, log_level="info",
                                loop="uvloop", workers=1)
    server = uvicorn.Server(server_cfg)

    import asyncio
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
