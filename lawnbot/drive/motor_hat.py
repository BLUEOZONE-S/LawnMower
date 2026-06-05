"""Waveshare Motor Driver HAT (15364) — TB6612FNG over PCA9685.

Wraps the PCA9685 in motor-shaped semantics. The HAT drives 2 brushed motors;
on this rover both motors share one throttle command (AWD) so the public API
is ``set_throttle(signed_duty)``. Sign → direction, magnitude → PWM duty.

Includes a software command-timeout watchdog: if ``set_throttle`` isn't called
within ``timeout_ms``, both motors auto-stop. This catches a crashed control
loop without needing a separate hardware kill.

Direction logic per TB6612 channel:
  forward: IN1=full-on,  IN2=full-off
  reverse: IN1=full-off, IN2=full-on
  brake:   IN1=full-on,  IN2=full-on
  coast:   IN1=full-off, IN2=full-off
Speed = duty on the channel's PWM pin (0..1).
"""
from __future__ import annotations

import threading
import time
from dataclasses import dataclass

from ..config import DriveCfg, MotorChannels
from .pca9685 import PCA9685


@dataclass
class MotorState:
    duty: float = 0.0
    forward: bool = True


class MotorHAT:
    def __init__(self, cfg: DriveCfg, pca: PCA9685 | None = None, timeout_ms: int = 300):
        self.cfg = cfg
        self.pca = pca or PCA9685(addr=cfg.pca9685_addr, pwm_hz=cfg.pwm_hz)
        if self.pca.pwm_hz != cfg.pwm_hz:
            self.pca.set_pwm_hz(cfg.pwm_hz)

        self.timeout_s = timeout_ms / 1000.0
        self._lock = threading.Lock()
        self._last_cmd = time.monotonic()
        self._rear = MotorState()
        self._front = MotorState()

        self._wd_stop = threading.Event()
        self._wd_thread = threading.Thread(target=self._watchdog, daemon=True)
        self._wd_thread.start()
        self.stop()

    # ---- public API -----------------------------------------------------

    def set_throttle(self, duty: float) -> None:
        """Drive both motors with the same signed duty in [-1, +1]."""
        duty = -1.0 if duty < -1 else 1.0 if duty > 1 else duty
        with self._lock:
            self._last_cmd = time.monotonic()
            self._apply(self.cfg.motor_rear, duty, invert=self.cfg.invert_rear, state=self._rear)
            if self.cfg.drive_both:
                self._apply(
                    self.cfg.motor_front, duty, invert=self.cfg.invert_front, state=self._front
                )
            else:
                self._coast(self.cfg.motor_front, self._front)

    def set_each(self, rear: float, front: float) -> None:
        """Drive motors independently. Used by motor_calibrate.py."""
        rear = -1.0 if rear < -1 else 1.0 if rear > 1 else rear
        front = -1.0 if front < -1 else 1.0 if front > 1 else front
        with self._lock:
            self._last_cmd = time.monotonic()
            self._apply(self.cfg.motor_rear, rear, invert=self.cfg.invert_rear, state=self._rear)
            self._apply(
                self.cfg.motor_front, front, invert=self.cfg.invert_front, state=self._front
            )

    def stop(self) -> None:
        """Coast both motors. Safe to call from any thread."""
        with self._lock:
            self._coast(self.cfg.motor_rear, self._rear)
            self._coast(self.cfg.motor_front, self._front)

    def brake(self) -> None:
        """Active brake — both IN pins high, PWM full-on."""
        with self._lock:
            self._brake(self.cfg.motor_rear, self._rear)
            self._brake(self.cfg.motor_front, self._front)

    def close(self) -> None:
        self._wd_stop.set()
        self.stop()
        self.pca.close()

    @property
    def state(self) -> dict:
        with self._lock:
            return {
                "rear": {"duty": self._rear.duty, "forward": self._rear.forward},
                "front": {"duty": self._front.duty, "forward": self._front.forward},
                "age_s": time.monotonic() - self._last_cmd,
            }

    # ---- internals ------------------------------------------------------

    def _apply(self, ch: MotorChannels, duty: float, invert: bool, state: MotorState) -> None:
        forward = (duty >= 0) ^ invert
        mag = abs(duty)
        if mag < 1e-4:
            self._coast(ch, state)
            return
        if forward:
            self.pca.full_on(ch.in1_ch)
            self.pca.full_off(ch.in2_ch)
        else:
            self.pca.full_off(ch.in1_ch)
            self.pca.full_on(ch.in2_ch)
        self.pca.set_duty(ch.pwm_ch, mag)
        state.duty = mag
        state.forward = forward

    def _coast(self, ch: MotorChannels, state: MotorState) -> None:
        self.pca.full_off(ch.in1_ch)
        self.pca.full_off(ch.in2_ch)
        self.pca.set_duty(ch.pwm_ch, 0.0)
        state.duty = 0.0

    def _brake(self, ch: MotorChannels, state: MotorState) -> None:
        self.pca.full_on(ch.in1_ch)
        self.pca.full_on(ch.in2_ch)
        self.pca.set_duty(ch.pwm_ch, 1.0)
        state.duty = 0.0

    def _watchdog(self) -> None:
        # Polls every ~50 ms; cheap, runs as daemon thread.
        while not self._wd_stop.is_set():
            with self._lock:
                stale = (time.monotonic() - self._last_cmd) > self.timeout_s
                moving = self._rear.duty > 0 or self._front.duty > 0
            if stale and moving:
                self.stop()
            time.sleep(0.05)
