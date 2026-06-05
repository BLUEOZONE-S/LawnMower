"""Pure-pursuit + PID heading-trim controller.

Pure-pursuit is the primary controller for Ackermann:
  δ = atan2(2 L sin(α), Ld)

where α = angle from rover heading to a lookahead point Ld meters ahead on
the path, and L is the wheelbase. Sparse 1 Hz GPS tolerates this far better
than point-chasing.

The PID provides an optional heading-error inner trim. Anti-windup is a
clamp on the I-term magnitude; saturation is the configured `steer_max`.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

from ..config import ControlCfg, GeometryCfg
from ..estimator import Pose


Point = tuple[float, float]


def wrap_pi(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def lookahead_point(path: list[Point], pose: Pose, ld: float) -> tuple[Point, int]:
    """Find the first point on `path` that is at least `ld` meters from pose.

    Returns (lookahead_point, index_used). Falls back to the last point if
    none is far enough — that pulls the rover toward the path's end.
    """
    if not path:
        return (pose.x, pose.y), -1
    best = path[-1]
    best_i = len(path) - 1
    for i, p in enumerate(path):
        dx, dy = p[0] - pose.x, p[1] - pose.y
        if math.hypot(dx, dy) >= ld:
            best = p
            best_i = i
            break
    return best, best_i


def pure_pursuit_steer(pose: Pose, target: Point, wheelbase: float) -> float:
    """Compute Ackermann steering angle δ pointing at `target`."""
    dx = target[0] - pose.x
    dy = target[1] - pose.y
    # Angle from world +x to the target.
    bearing = math.atan2(dy, dx)
    alpha = wrap_pi(bearing - pose.theta)
    ld = math.hypot(dx, dy)
    if ld < 1e-3:
        return 0.0
    return math.atan2(2.0 * wheelbase * math.sin(alpha), ld)


@dataclass
class PIDState:
    integ: float = 0.0
    prev_err: float = 0.0
    last_p: float = 0.0
    last_i: float = 0.0
    last_d: float = 0.0


class HeadingPID:
    """PID on heading error → steering trim (radians)."""

    def __init__(self, cfg: ControlCfg):
        self.cfg = cfg
        self.state = PIDState()

    def reset(self) -> None:
        self.state = PIDState()

    def step(self, heading_err: float, dt: float) -> float:
        kp, ki, kd = self.cfg.pid.kp, self.cfg.pid.ki, self.cfg.pid.kd
        imax = self.cfg.pid.imax
        s = self.state
        # Anti-windup: clamp integrator.
        s.integ = max(-imax, min(imax, s.integ + heading_err * dt))
        deriv = (heading_err - s.prev_err) / dt if dt > 1e-6 else 0.0
        s.prev_err = heading_err
        p = kp * heading_err
        i = ki * s.integ
        d = kd * deriv
        s.last_p, s.last_i, s.last_d = p, i, d
        return p + i + d

    @property
    def breakdown(self) -> dict:
        s = self.state
        return {"p": s.last_p, "i": s.last_i, "d": s.last_d}


@dataclass
class ControlOutput:
    v: float
    delta: float
    lookahead_idx: int
    heading_err: float
    cross_track: float


class Controller:
    def __init__(self, cfg: ControlCfg, geom: GeometryCfg):
        self.cfg = cfg
        self.geom = geom
        self.pid = HeadingPID(cfg)

    def step(self, pose: Pose, path: list[Point], dt: float) -> ControlOutput:
        if not path:
            return ControlOutput(v=0.0, delta=0.0, lookahead_idx=-1, heading_err=0.0, cross_track=0.0)
        target, idx = lookahead_point(path, pose, self.cfg.pure_pursuit.lookahead_m)

        # Steering = pure-pursuit (+ PID trim on heading error to the immediate next waypoint).
        delta_pp = pure_pursuit_steer(pose, target, self.geom.wheelbase_m)

        next_wp = path[min(idx, len(path) - 1)] if idx >= 0 else path[-1]
        bearing_next = math.atan2(next_wp[1] - pose.y, next_wp[0] - pose.x)
        heading_err = wrap_pi(bearing_next - pose.theta)
        trim = self.pid.step(heading_err, dt)
        delta = max(-self.geom.steer_max_rad, min(self.geom.steer_max_rad, delta_pp + trim))

        # Cross-track approximation: perpendicular distance from pose to the
        # line through the next waypoint heading along path tangent.
        cross_track = 0.0
        if idx > 0:
            prev = path[idx - 1]
            cur = path[idx]
            tx, ty = cur[0] - prev[0], cur[1] - prev[1]
            tn = math.hypot(tx, ty) or 1e-9
            tx, ty = tx / tn, ty / tn
            px, py = pose.x - prev[0], pose.y - prev[1]
            cross_track = -tx * py + ty * px  # signed

        return ControlOutput(
            v=self.cfg.v_nominal,
            delta=delta,
            lookahead_idx=idx,
            heading_err=heading_err,
            cross_track=cross_track,
        )

    def update_gains(self, kp: float | None = None, ki: float | None = None,
                     kd: float | None = None, lookahead_m: float | None = None,
                     v_nominal: float | None = None) -> None:
        """Live-tune from the UI. Replaces the cfg in-place via dataclass copy."""
        from dataclasses import replace
        pid = replace(self.cfg.pid,
                      kp=kp if kp is not None else self.cfg.pid.kp,
                      ki=ki if ki is not None else self.cfg.pid.ki,
                      kd=kd if kd is not None else self.cfg.pid.kd)
        pp = replace(self.cfg.pure_pursuit,
                     lookahead_m=lookahead_m if lookahead_m is not None else self.cfg.pure_pursuit.lookahead_m)
        self.cfg = replace(self.cfg, pid=pid, pure_pursuit=pp,
                           v_nominal=v_nominal if v_nominal is not None else self.cfg.v_nominal)
        self.pid.cfg = self.cfg
