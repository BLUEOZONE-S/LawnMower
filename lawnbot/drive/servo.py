"""Steering servo via pigpiod hardware-timed PWM.

The PCA9685 on the Motor HAT can't carry the servo (one frequency for all
channels — motors want ~1 kHz, servos want 50 Hz). The servo signal sits on
a Pi GPIO and is driven by the pigpio daemon for jitter-free timing.

set_steer(delta_rad): clamps to [-steer_max, +steer_max], asymmetric mapping
to pulse width so the calibrated us_center / us_min / us_max don't have to be
mathematically symmetric around 1500 us — the servo's mechanical center is
rarely exactly 1500 us.
"""
from __future__ import annotations

import math

try:
    import pigpio
except ImportError:  # syntax-check on dev machines
    pigpio = None  # type: ignore

from ..config import GeometryCfg, SteeringCfg


class SteeringServo:
    def __init__(self, cfg: SteeringCfg, geom: GeometryCfg):
        if pigpio is None:
            raise RuntimeError("pigpio not available — install on the Pi or run in dev mode")
        self.cfg = cfg
        self.geom = geom
        self._pi = pigpio.pi()
        if not self._pi.connected:
            raise RuntimeError("pigpiod is not running (sudo systemctl start pigpiod)")
        # pigpio uses set_servo_pulsewidth which is hardware-timed at 50 Hz.
        self.center()

    def set_steer(self, delta_rad: float) -> None:
        """delta_rad: positive = right (or left if invert=True)."""
        dmax = self.geom.steer_max_rad
        if delta_rad > dmax:
            delta_rad = dmax
        elif delta_rad < -dmax:
            delta_rad = -dmax
        if self.cfg.invert:
            delta_rad = -delta_rad

        if delta_rad >= 0:
            us = self.cfg.us_center + (delta_rad / dmax) * (self.cfg.us_max - self.cfg.us_center)
        else:
            # delta_rad is negative; (us_center - us_min) is positive; product is negative ⇒ us < us_center.
            us = self.cfg.us_center + (delta_rad / dmax) * (self.cfg.us_center - self.cfg.us_min)

        us = max(self.cfg.us_min, min(self.cfg.us_max, int(round(us))))
        self._pi.set_servo_pulsewidth(self.cfg.gpio_pin, us)

    def set_us(self, us: int) -> None:
        """Direct pulse-width — used by the steering-calibration CLI."""
        us = max(self.cfg.us_min, min(self.cfg.us_max, int(us)))
        self._pi.set_servo_pulsewidth(self.cfg.gpio_pin, us)

    def center(self) -> None:
        self._pi.set_servo_pulsewidth(self.cfg.gpio_pin, self.cfg.us_center)

    def release(self) -> None:
        """Stop sending pulses — servo holds last position but draws no torque."""
        self._pi.set_servo_pulsewidth(self.cfg.gpio_pin, 0)

    def close(self) -> None:
        try:
            self.release()
        finally:
            self._pi.stop()
