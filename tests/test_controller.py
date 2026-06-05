import math

from lawnbot.config import ControlCfg, GeometryCfg, PIDGains, PurePursuit
from lawnbot.estimator import Pose
from lawnbot.nav.controller import Controller, lookahead_point, pure_pursuit_steer, wrap_pi


def _ctrl():
    return ControlCfg(
        ctrl_hz=20,
        v_nominal=0.45,
        reach_m=0.22,
        max_steer_deg=30,
        pure_pursuit=PurePursuit(enabled=True, lookahead_m=0.6),
        pid=PIDGains(kp=3.0, ki=0.3, kd=0.35, imax=1.0),
    )


def _geom():
    return GeometryCfg(wheelbase_m=0.25, steer_max_deg=30, deck_m=0.5, body_clearance_m=0.12)


def test_wrap_pi():
    # atan2 returns values in (-π, π]; 3π wraps to +π (the boundary).
    assert math.isclose(wrap_pi(math.pi * 3), math.pi, abs_tol=1e-9)
    assert math.isclose(wrap_pi(0.1), 0.1)
    assert math.isclose(wrap_pi(-math.pi - 0.5), math.pi - 0.5, abs_tol=1e-9)


def test_lookahead_picks_first_far_enough():
    pose = Pose(0, 0, 0)
    path = [(0.1, 0), (0.3, 0), (0.7, 0), (1.5, 0)]
    pt, idx = lookahead_point(path, pose, 0.5)
    assert idx == 2
    assert pt == (0.7, 0)


def test_pure_pursuit_zero_when_aligned():
    pose = Pose(0, 0, 0)
    delta = pure_pursuit_steer(pose, (1, 0), wheelbase=0.25)
    assert abs(delta) < 1e-9


def test_pure_pursuit_steers_toward_lateral_target():
    pose = Pose(0, 0, 0)
    delta_left = pure_pursuit_steer(pose, (1, 1), 0.25)
    delta_right = pure_pursuit_steer(pose, (1, -1), 0.25)
    assert delta_left > 0
    assert delta_right < 0
    assert math.isclose(delta_left, -delta_right, abs_tol=1e-9)


def test_controller_steer_clamps():
    c = Controller(_ctrl(), _geom())
    pose = Pose(0, 0, 0)
    # Path hard left — should saturate.
    path = [(0.1, 5.0), (0.2, 5.0)]
    out = c.step(pose, path, dt=0.05)
    assert abs(out.delta) <= _geom().steer_max_rad + 1e-9
    assert abs(out.delta) > _geom().steer_max_rad * 0.8  # near full lock
