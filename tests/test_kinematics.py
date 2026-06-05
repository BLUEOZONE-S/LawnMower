"""Pure-math tests — run on the Windows dev box, no hardware needed.

    python -m pytest tests/
"""
from __future__ import annotations

import math

from lawnbot.config import DriveCfg, GeometryCfg, MotorChannels
from lawnbot.drive.kinematics import fits_turn, shape_speed, vd_to_command, yaw_rate


def _drive() -> DriveCfg:
    return DriveCfg(
        pca9685_addr=0x40,
        pwm_hz=1000,
        motor_rear=MotorChannels(0, 1, 2),
        motor_front=MotorChannels(5, 3, 4),
        drive_both=True,
        v_max=1.2,
        deadband=0.08,
        invert_rear=False,
        invert_front=False,
    )


def _geom() -> GeometryCfg:
    return GeometryCfg(wheelbase_m=0.25, steer_max_deg=30, deck_m=0.5, body_clearance_m=0.12)


def test_shape_speed_floors_at_30pct():
    v = 1.0
    eased = shape_speed(v, math.radians(80))
    assert eased == 0.3


def test_zero_steer_full_speed():
    assert shape_speed(1.0, 0.0) == 1.0


def test_steer_clamps_at_max():
    cmd = vd_to_command(0.5, math.radians(45), _drive(), _geom())
    assert math.isclose(abs(cmd.steer_rad), _geom().steer_max_rad)


def test_throttle_deadband_pushes_up_small_commands():
    drive = _drive()
    # ~3% of v_max = duty ≈ 0.03; should be bumped to deadband.
    cmd = vd_to_command(0.03 * drive.v_max, 0.0, drive, _geom())
    assert math.isclose(cmd.throttle_duty, drive.deadband)


def test_throttle_zero_stays_zero():
    cmd = vd_to_command(0.0, 0.0, _drive(), _geom())
    assert cmd.throttle_duty == 0.0


def test_yaw_rate_matches_bicycle_model():
    g = _geom()
    v, d = 1.0, math.radians(20)
    assert math.isclose(yaw_rate(v, d, g), v * math.tan(d) / g.wheelbase_m)


def test_min_turn_radius():
    g = _geom()
    expected = g.wheelbase_m / math.tan(g.steer_max_rad)
    assert math.isclose(g.min_turn_radius_m, expected)
    assert fits_turn(expected * 1.1, g) is True
    assert fits_turn(expected * 0.9, g) is False
