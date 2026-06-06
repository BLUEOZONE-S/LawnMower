"""Steering servo via lgpio hardware-timed PWM.

The PCA9685 on the Motor HAT can't carry the servo (one frequency for all
channels — motors want ~1 kHz, servos want 50 Hz). The servo signal sits on
a Pi GPIO and is driven by lgpio's tx_servo for jitter-free timing.

set_steer(delta_rad): clamps to [-steer_max, +steer_max], asymmetric mapping
to pulse width so the calibrated us_center / us_min / us_max don't have to be
mathematically symmetric around 1500 us — the servo's mechanical center is
rarely exactly 1500 us.

Chip selection: Pi 5's RP1 exposes header GPIO on /dev/gpiochip4; Zero 2 W /
Pi 4 expose them on /dev/gpiochip0. We try 4 first, then 0, so the same code
works across boards.
"""
from __future__ import annotations

import os

try:
    import lgpio
except ImportError:  # syntax-check on dev machines
    lgpio = None  # type: ignore

from ..config import GeometryCfg, SteeringCfg


def _open_header_gpiochip():
    last_err: Exception | None = None
    for n in (4, 0):
        if not os.path.exists(f"/dev/gpiochip{n}"):
            continue
        try:
            return lgpio.gpiochip_open(n)
        except Exception as e:  # noqa: BLE001
            last_err = e
    raise RuntimeError(
        f"could not open /dev/gpiochip4 or /dev/gpiochip0 ({last_err})"
    )


class SteeringServo:
    def __init__(self, cfg: SteeringCfg, geom: GeometryCfg):
        if lgpio is None:
            raise RuntimeError(
                "lgpio not available — `sudo apt install python3-lgpio` "
                "or `pip install lgpio` in the venv"
            )
        self.cfg = cfg
        self.geom = geom
        self._h = _open_header_gpiochip()
        # tx_servo auto-claims the GPIO as a PWM output, so no explicit claim.
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
            us = self.cfg.us_center + (delta_rad / dmax) * (self.cfg.us_center - self.cfg.us_min)

        us = max(self.cfg.us_min, min(self.cfg.us_max, int(round(us))))
        lgpio.tx_servo(self._h, self.cfg.gpio_pin, us)

    def set_us(self, us: int) -> None:
        """Direct pulse-width — used by the steering-calibration CLI."""
        us = max(self.cfg.us_min, min(self.cfg.us_max, int(us)))
        lgpio.tx_servo(self._h, self.cfg.gpio_pin, us)

    def center(self) -> None:
        lgpio.tx_servo(self._h, self.cfg.gpio_pin, self.cfg.us_center)

    def release(self) -> None:
        """Stop sending pulses — servo holds last position but draws no torque."""
        lgpio.tx_servo(self._h, self.cfg.gpio_pin, 0)

    def close(self) -> None:
        try:
            self.release()
        finally:
            try:
                lgpio.gpiochip_close(self._h)
            except Exception:
                pass
