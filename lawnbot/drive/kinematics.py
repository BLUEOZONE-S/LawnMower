"""Ackermann / bicycle kinematics.

The platform is an RC car: two brushed drive motors share one throttle and
a single front servo sets the steering angle delta. This module converts
controller outputs (forward speed v, steering angle delta) into actuator
commands (throttle duty for both motors, servo pulse-width through the
servo driver).

Geometry:
  yaw_rate omega = v * tan(delta) / wheelbase
  min turning radius R_min = wheelbase / tan(steer_max)

Speed shaping: v is multiplied by max(0.3, cos(delta)) so the rover slows
into tight steering — keeps the controller honest about R_min and reduces
slip at full lock.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

from ..config import DriveCfg, GeometryCfg


@dataclass
class DriveCommand:
    throttle_duty: float  # signed, [-1, +1] — to both motors
    steer_rad: float  # clamped to [-steer_max, +steer_max]
    v_commanded: float  # m/s, post-shaping (for telemetry)


def shape_speed(v: float, delta_rad: float) -> float:
    """Ease off speed as steering approaches lock. Floor = 30% of v."""
    return v * max(0.3, math.cos(delta_rad))


def vd_to_command(v: float, delta_rad: float, drive: DriveCfg, geom: GeometryCfg) -> DriveCommand:
    """Map (speed, steer) → throttle duty + clamped steer.

    Applies the speed-shaping in cos(delta), clamps steer at steer_max, and
    applies a symmetric deadband to the throttle so the motors don't sit at
    a duty too low to move.
    """
    delta_rad = max(-geom.steer_max_rad, min(geom.steer_max_rad, delta_rad))
    v_shaped = shape_speed(v, delta_rad)

    # Convert speed to duty using calibrated v_max.
    duty = v_shaped / drive.v_max
    duty = max(-1.0, min(1.0, duty))

    # Deadband: anything below the deadband can't actually move the rover.
    # Push small commands up to the deadband (preserving sign) so the rover
    # actually rolls; zero out commands that are essentially zero already.
    db = drive.deadband
    if abs(duty) < 1e-3:
        duty = 0.0
    elif 0 < abs(duty) < db:
        duty = math.copysign(db, duty)

    return DriveCommand(throttle_duty=duty, steer_rad=delta_rad, v_commanded=v_shaped)


def yaw_rate(v: float, delta_rad: float, geom: GeometryCfg) -> float:
    """Bicycle-model yaw rate given the commanded speed + steer."""
    return v * math.tan(delta_rad) / geom.wheelbase_m


def fits_turn(radius_m: float, geom: GeometryCfg) -> bool:
    """True if a turn of the given radius is within the rover's R_min."""
    return radius_m >= geom.min_turn_radius_m
