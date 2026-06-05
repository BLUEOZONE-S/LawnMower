"""Manual teleop with deadman heartbeat.

The operator sends teleop commands (v, delta) from the web UI's joystick /
keyboard / gamepad. Every command carries a timestamp; if none arrives
within `deadman_ms`, motors stop. This reuses the motor command-timeout —
the deadman simply stops calling set_throttle.

Geofence is still enforced in MANUAL (warn + stop at the fence) unless an
explicit override is held.
"""
from __future__ import annotations

import threading
import time
from dataclasses import dataclass

from .config import GeometryCfg, TeleopCfg
from .drive.kinematics import vd_to_command


@dataclass
class TeleopCommand:
    v: float = 0.0  # m/s, signed
    delta: float = 0.0  # rad, signed (clamped at manual_steer_deg)
    override_geofence: bool = False
    last_seen_mono: float = 0.0


class Teleop:
    def __init__(self, cfg: TeleopCfg, geom: GeometryCfg):
        self.cfg = cfg
        self.geom = geom
        self._lock = threading.Lock()
        self._cmd = TeleopCommand()
        # Effective manual cap (radians)
        self._steer_cap = min(geom.steer_max_rad, _deg_to_rad(cfg.manual_steer_deg))
        self._v_cap = cfg.manual_v_max

    def ingest(self, v: float, delta_rad: float, override_geofence: bool = False) -> None:
        """Called from the WebSocket handler on every operator command."""
        v = max(-self._v_cap, min(self._v_cap, float(v)))
        delta = max(-self._steer_cap, min(self._steer_cap, float(delta_rad)))
        with self._lock:
            self._cmd = TeleopCommand(
                v=v,
                delta=delta,
                override_geofence=bool(override_geofence),
                last_seen_mono=time.monotonic(),
            )

    def latest(self) -> tuple[TeleopCommand, bool]:
        """Returns (cmd, alive). alive=False means deadman expired."""
        with self._lock:
            cmd = TeleopCommand(**self._cmd.__dict__)
        age_ms = (time.monotonic() - cmd.last_seen_mono) * 1000
        alive = cmd.last_seen_mono > 0 and age_ms < self.cfg.deadman_ms
        return cmd, alive

    def command_for_motors(self, drive_cfg) -> tuple[float, float] | None:
        """Convert latest teleop command → (throttle_duty, steer_rad).

        Returns None if deadman expired (caller should stop motors).
        """
        cmd, alive = self.latest()
        if not alive:
            return None
        out = vd_to_command(cmd.v, cmd.delta, drive_cfg, self.geom)
        return out.throttle_duty, out.steer_rad


def _deg_to_rad(d: float) -> float:
    import math
    return math.radians(d)
