"""Config loader. Single source of truth for pins, gains, thresholds.

Reads config.yaml once at startup into a tree of frozen dataclasses, so
downstream modules can take a typed handle instead of dict-diving.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from pathlib import Path

import yaml


@dataclass(frozen=True)
class MotorChannels:
    pwm_ch: int
    in1_ch: int
    in2_ch: int


@dataclass(frozen=True)
class PurePursuit:
    enabled: bool
    lookahead_m: float


@dataclass(frozen=True)
class PIDGains:
    kp: float
    ki: float
    kd: float
    imax: float


@dataclass(frozen=True)
class ControlCfg:
    ctrl_hz: int
    v_nominal: float
    reach_m: float
    max_steer_deg: float
    pure_pursuit: PurePursuit
    pid: PIDGains

    @property
    def max_steer_rad(self) -> float:
        return math.radians(self.max_steer_deg)


@dataclass(frozen=True)
class GeometryCfg:
    wheelbase_m: float
    steer_max_deg: float
    deck_m: float
    body_clearance_m: float

    @property
    def steer_max_rad(self) -> float:
        return math.radians(self.steer_max_deg)

    @property
    def min_turn_radius_m(self) -> float:
        return self.wheelbase_m / math.tan(self.steer_max_rad)


@dataclass(frozen=True)
class DriveCfg:
    pca9685_addr: int
    pwm_hz: int
    motor_rear: MotorChannels
    motor_front: MotorChannels
    drive_both: bool
    v_max: float
    deadband: float
    invert_rear: bool
    invert_front: bool


@dataclass(frozen=True)
class SteeringCfg:
    gpio_pin: int
    pwm_hz: int
    us_center: int
    us_min: int
    us_max: int
    invert: bool


@dataclass(frozen=True)
class GnssCfg:
    port: str
    baud: int
    min_fix_quality: int
    fix_max_age_s: float


@dataclass(frozen=True)
class NtripCfg:
    host: str
    port: int
    mountpoint: str
    user: str
    password: str


@dataclass(frozen=True)
class EstimatorCfg:
    gps_blend_alpha: float
    imu_yaw_offset_deg: float


@dataclass(frozen=True)
class SafetyCfg:
    geo_margin_m: float
    return_pct: int
    stop_pct: int
    degrade_sec: float
    motor_timeout_ms: int


@dataclass(frozen=True)
class UICfg:
    port: int
    push_hz: int


@dataclass(frozen=True)
class TeleopCfg:
    deadman_ms: int
    manual_v_max: float
    manual_steer_deg: float
    enforce_geofence: bool


@dataclass(frozen=True)
class StuckCfg:
    window_s: float
    min_progress_m: float
    recovery_tries: int
    giveup_s: float


@dataclass(frozen=True)
class TeachCfg:
    sample_m: float
    simplify_m: float
    loop_min_perimeter_m: float
    loop_closure_m: float
    auto_confirm_closure: bool
    boundary_inset_m: float


@dataclass(frozen=True)
class Config:
    platform: str
    control: ControlCfg
    geometry: GeometryCfg
    drive: DriveCfg
    steering: SteeringCfg
    gnss: GnssCfg
    ntrip: NtripCfg
    estimator: EstimatorCfg
    safety: SafetyCfg
    ui: UICfg
    teleop: TeleopCfg
    stuck: StuckCfg
    teach: TeachCfg


def _channels(d: dict) -> MotorChannels:
    return MotorChannels(pwm_ch=d["pwm_ch"], in1_ch=d["in1_ch"], in2_ch=d["in2_ch"])


def load(path: str | Path = "config.yaml") -> Config:
    with open(path, "r", encoding="utf-8") as f:
        raw = yaml.safe_load(f)

    c = raw["control"]
    return Config(
        platform=raw["platform"],
        control=ControlCfg(
            ctrl_hz=c["ctrl_hz"],
            v_nominal=c["v_nominal"],
            reach_m=c["reach_m"],
            max_steer_deg=c["max_steer_deg"],
            pure_pursuit=PurePursuit(**c["pure_pursuit"]),
            pid=PIDGains(**c["pid"]),
        ),
        geometry=GeometryCfg(**raw["geometry"]),
        drive=DriveCfg(
            pca9685_addr=int(raw["drive"]["pca9685_addr"]),
            pwm_hz=raw["drive"]["pwm_hz"],
            motor_rear=_channels(raw["drive"]["motor_rear"]),
            motor_front=_channels(raw["drive"]["motor_front"]),
            drive_both=raw["drive"]["drive_both"],
            v_max=raw["drive"]["v_max"],
            deadband=raw["drive"]["deadband"],
            invert_rear=raw["drive"].get("invert_rear", False),
            invert_front=raw["drive"].get("invert_front", False),
        ),
        steering=SteeringCfg(**raw["steering"]),
        gnss=GnssCfg(**raw["gnss"]),
        ntrip=NtripCfg(**raw["ntrip"]),
        estimator=EstimatorCfg(**raw["estimator"]),
        safety=SafetyCfg(**raw["safety"]),
        ui=UICfg(**raw["ui"]),
        teleop=TeleopCfg(**raw["teleop"]),
        stuck=StuckCfg(**raw["stuck"]),
        teach=TeachCfg(**raw["teach"]),
    )
