"""Minimal PCA9685 16-channel PWM driver over I2C (smbus2).

Used by the Waveshare 15364 HAT. Kept dependency-free of Adafruit Blinka to
stay light on the Pi Zero 2 W. One PCA9685 = one shared frequency for all
16 channels — that's why the steering servo lives on a GPIO instead.
"""
from __future__ import annotations

import time

try:
    from smbus2 import SMBus
except ImportError:  # syntax-check on dev machines without smbus2
    SMBus = None  # type: ignore

# PCA9685 register map
_MODE1 = 0x00
_MODE2 = 0x01
_PRESCALE = 0xFE
_LED0_ON_L = 0x06  # ch N base = _LED0_ON_L + 4*N

# MODE1 bits
_MODE1_RESTART = 0x80
_MODE1_SLEEP = 0x10
_MODE1_AI = 0x20  # auto-increment

# MODE2 bits
_MODE2_OUTDRV = 0x04  # totem-pole output

_PWM_FULL = 4096  # 12-bit + the "full on" bit


class PCA9685:
    """Direct PCA9685 control. Thread-safe per instance via the I2C bus lock."""

    def __init__(self, addr: int = 0x40, busnum: int = 1, pwm_hz: int = 1000):
        if SMBus is None:
            raise RuntimeError("smbus2 not available — install on the Pi or run in dev mode")
        self.addr = addr
        self.bus = SMBus(busnum)
        self._reset()
        self.set_pwm_hz(pwm_hz)

    def _write8(self, reg: int, val: int) -> None:
        self.bus.write_byte_data(self.addr, reg, val & 0xFF)

    def _read8(self, reg: int) -> int:
        return self.bus.read_byte_data(self.addr, reg) & 0xFF

    def _reset(self) -> None:
        self._write8(_MODE2, _MODE2_OUTDRV)
        self._write8(_MODE1, _MODE1_AI)
        time.sleep(0.005)

    def set_pwm_hz(self, hz: int) -> None:
        """Datasheet: prescale = round(25 MHz / (4096 * hz)) - 1."""
        prescale = max(3, min(255, int(round(25_000_000.0 / (4096 * hz)) - 1)))
        old = self._read8(_MODE1)
        # Must go to sleep to change prescale.
        self._write8(_MODE1, (old & ~_MODE1_RESTART) | _MODE1_SLEEP)
        self._write8(_PRESCALE, prescale)
        self._write8(_MODE1, old)
        time.sleep(0.005)
        self._write8(_MODE1, old | _MODE1_RESTART | _MODE1_AI)
        self.pwm_hz = hz

    def set_pwm(self, channel: int, on: int, off: int) -> None:
        """Set raw on/off counts (0..4096) on a channel."""
        base = _LED0_ON_L + 4 * channel
        self.bus.write_i2c_block_data(
            self.addr, base, [on & 0xFF, (on >> 8) & 0x1F, off & 0xFF, (off >> 8) & 0x1F]
        )

    def set_duty(self, channel: int, duty: float) -> None:
        """duty in [0.0, 1.0]. Convenience over set_pwm."""
        duty = 0.0 if duty < 0 else 1.0 if duty > 1 else duty
        if duty <= 0.0:
            self.set_pwm(channel, 0, _PWM_FULL)  # full-off
        elif duty >= 1.0:
            self.set_pwm(channel, _PWM_FULL, 0)  # full-on
        else:
            self.set_pwm(channel, 0, int(round(duty * 4095)))

    def full_on(self, channel: int) -> None:
        self.set_pwm(channel, _PWM_FULL, 0)

    def full_off(self, channel: int) -> None:
        self.set_pwm(channel, 0, _PWM_FULL)

    def all_off(self) -> None:
        for ch in range(16):
            self.full_off(ch)

    def close(self) -> None:
        try:
            self.all_off()
        finally:
            self.bus.close()
