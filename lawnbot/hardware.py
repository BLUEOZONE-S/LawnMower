"""Hardware factory — picks real or simulated drivers from config.

Keeps :mod:`lawnbot.main` agnostic of which backend it talks to. In sim
mode the leaf drivers all reference a shared :class:`SimWorld`; in real
mode they hit I2C / UART / pigpio as usual.
"""
from __future__ import annotations

import logging
from dataclasses import dataclass
from typing import Any

from .config import Config
from .nav.geo import Origin


log = logging.getLogger("lawnbot.hardware")


@dataclass
class Hardware:
    motors: Any
    servo: Any
    gps: Any
    imu: Any
    odom: Any
    pisugar: Any
    ntrip: Any
    world: Any = None  # populated in sim mode so Runtime can stop it on shutdown


def make_hardware(cfg: Config) -> Hardware:
    if cfg.sim.enabled:
        log.info("hardware backend: SIMULATION")
        return _build_sim(cfg)
    log.info("hardware backend: real Pi peripherals")
    return _build_real(cfg)


def _build_real(cfg: Config) -> Hardware:
    from .drive.motor_hat import MotorHAT
    from .drive.servo import SteeringServo
    from .gnss.lc29h import LC29H
    from .gnss.ntrip import NtripForwarder
    from .power.pisugar import PiSugar
    from .sensors.imu import IMU, StubIMU
    from .sensors.odometry import StubOdometry

    motors = MotorHAT(cfg.drive, timeout_ms=cfg.safety.motor_timeout_ms)
    servo = SteeringServo(cfg.steering, cfg.geometry)
    servo.center()

    gps = LC29H(cfg.gnss)

    try:
        imu: Any = IMU()
    except Exception as exc:
        log.warning("IMU unavailable (%s) — using StubIMU", exc)
        imu = StubIMU()

    odom = StubOdometry()
    pisugar = PiSugar()
    ntrip = NtripForwarder(cfg.ntrip, gps.write_rtcm)
    return Hardware(motors, servo, gps, imu, odom, pisugar, ntrip)


def _build_sim(cfg: Config) -> Hardware:
    from .sim.drivers import (
        SimIMU,
        SimLC29H,
        SimMotorHAT,
        SimNtrip,
        SimOdometry,
        SimPiSugar,
        SimSteeringServo,
    )
    from .sim.world import SimWorld

    origin = Origin(lat=cfg.sim.origin_lat, lon=cfg.sim.origin_lon)
    world = SimWorld(cfg.sim, cfg.geometry, cfg.drive, origin)

    motors = SimMotorHAT(world, cfg.drive, cfg.safety.motor_timeout_ms)
    servo = SimSteeringServo(world, cfg.steering, cfg.geometry)
    servo.center()
    gps = SimLC29H(world, cfg.gnss, cfg.sim, origin)
    imu = SimIMU(world, cfg.sim)
    odom = SimOdometry(world)
    pisugar = SimPiSugar(world)
    ntrip = SimNtrip()
    return Hardware(motors, servo, gps, imu, odom, pisugar, ntrip, world=world)
