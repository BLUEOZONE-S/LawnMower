"""Absolute-yaw IMU wrapper.

Default target: BNO085 over I2C (Adafruit's Adafruit_CircuitPython_BNO08x lib).
The brief allows BNO055 as a substitute. The estimator just needs `yaw_rad()`
fast — anything that gives absolute yaw works.

Falls back to a stub on dev machines (Windows) so the package imports.
"""
from __future__ import annotations

import math
import threading
import time

try:
    import board  # adafruit-blinka
    import busio
    from adafruit_bno08x import BNO_REPORT_ROTATION_VECTOR  # type: ignore
    from adafruit_bno08x.i2c import BNO08X_I2C  # type: ignore
    _HAS_BNO = True
except ImportError:
    _HAS_BNO = False


class IMU:
    def __init__(self, i2c_addr: int = 0x4A):
        self._lock = threading.Lock()
        self._yaw = 0.0
        self._last_update = 0.0
        if _HAS_BNO:
            self._i2c = busio.I2C(board.SCL, board.SDA, frequency=400000)
            self._bno = BNO08X_I2C(self._i2c, address=i2c_addr)
            self._bno.enable_feature(BNO_REPORT_ROTATION_VECTOR)
        else:
            self._bno = None

    def yaw_rad(self) -> float:
        if self._bno is None:
            return 0.0
        try:
            qi, qj, qk, qr = self._bno.quaternion
        except Exception:
            return self._yaw
        # Extract yaw (Z) from a quaternion (i,j,k,r) where r is real.
        siny_cosp = 2.0 * (qr * qk + qi * qj)
        cosy_cosp = 1.0 - 2.0 * (qj * qj + qk * qk)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        with self._lock:
            self._yaw = yaw
            self._last_update = time.monotonic()
        return yaw

    @property
    def has_hardware(self) -> bool:
        return self._bno is not None


class StubIMU:
    """Always-zero yaw — for bring-up before the IMU is wired."""

    def yaw_rad(self) -> float:
        return 0.0

    @property
    def has_hardware(self) -> bool:
        return False
