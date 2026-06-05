"""Quadrature wheel-encoder odometry via pigpio callbacks.

Counts encoder ticks between calls to ``read_delta_m()`` and converts them
to meters using the calibrated wheel circumference + counts-per-revolution.

Sign is recovered from the quadrature pattern (A leads B vs B leads A).
If only a single channel is wired, configure single-channel mode and sign
will be inferred from the last commanded direction (less robust — wire
both channels if possible).
"""
from __future__ import annotations

import threading

try:
    import pigpio
except ImportError:
    pigpio = None  # type: ignore


class QuadratureEncoder:
    def __init__(
        self,
        pin_a: int,
        pin_b: int,
        counts_per_rev: int,
        wheel_circumference_m: float,
        invert: bool = False,
    ):
        if pigpio is None:
            raise RuntimeError("pigpio not available")
        self.pin_a = pin_a
        self.pin_b = pin_b
        self.counts_per_rev = counts_per_rev
        self.wheel_circ_m = wheel_circumference_m
        self.invert = invert

        self._pi = pigpio.pi()
        if not self._pi.connected:
            raise RuntimeError("pigpiod is not running")
        self._pi.set_mode(pin_a, pigpio.INPUT)
        self._pi.set_mode(pin_b, pigpio.INPUT)
        self._pi.set_pull_up_down(pin_a, pigpio.PUD_UP)
        self._pi.set_pull_up_down(pin_b, pigpio.PUD_UP)

        self._lock = threading.Lock()
        self._ticks = 0
        self._last_read_ticks = 0
        self._state = (self._pi.read(pin_a) << 1) | self._pi.read(pin_b)

        self._cb_a = self._pi.callback(pin_a, pigpio.EITHER_EDGE, self._on_edge)
        self._cb_b = self._pi.callback(pin_b, pigpio.EITHER_EDGE, self._on_edge)

    # ---- public API -----------------------------------------------------

    def read_delta_m(self) -> float:
        """Return distance traveled since last call."""
        with self._lock:
            delta_ticks = self._ticks - self._last_read_ticks
            self._last_read_ticks = self._ticks
        ds = (delta_ticks / self.counts_per_rev) * self.wheel_circ_m
        return -ds if self.invert else ds

    @property
    def ticks(self) -> int:
        with self._lock:
            return self._ticks

    def close(self) -> None:
        try:
            self._cb_a.cancel()
            self._cb_b.cancel()
        finally:
            self._pi.stop()

    # ---- internals ------------------------------------------------------

    # 4x decoding table: state transition → direction. Keys are (old, new).
    _DIR = {
        (0b00, 0b01): +1, (0b01, 0b11): +1, (0b11, 0b10): +1, (0b10, 0b00): +1,
        (0b00, 0b10): -1, (0b10, 0b11): -1, (0b11, 0b01): -1, (0b01, 0b00): -1,
    }

    def _on_edge(self, gpio, level, tick):  # noqa: ARG002 -- pigpio signature
        new_state = (self._pi.read(self.pin_a) << 1) | self._pi.read(self.pin_b)
        direction = self._DIR.get((self._state, new_state), 0)
        with self._lock:
            self._ticks += direction
        self._state = new_state


class StubOdometry:
    """No-encoder fallback. Returns zero — pose drifts on dead-reckon."""

    def read_delta_m(self) -> float:
        return 0.0

    def close(self) -> None:
        pass
