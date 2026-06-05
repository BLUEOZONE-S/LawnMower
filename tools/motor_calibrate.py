"""Phase-1 bench bring-up.

Drives each motor independently and sweeps the steering servo so you can:
  - confirm wiring (forward/reverse, left/right)
  - record us_min / us_center / us_max for the servo
  - estimate v_max once the rover is on the ground (in a later step)

CRITICAL: run this with the rover's wheels OFF THE GROUND (chassis on a
stand). A runaway duty value will otherwise send it across the room.

Usage on the Pi:
    sudo systemctl start pigpiod        # required for the servo
    python3 -m tools.motor_calibrate
"""
from __future__ import annotations

import sys
import time

from lawnbot import config
from lawnbot.drive.motor_hat import MotorHAT
from lawnbot.drive.servo import SteeringServo


HELP = """
Commands:
  rf [duty]   rear motor forward at duty (default 0.25)
  rr [duty]   rear motor reverse at duty
  ff [duty]   front motor forward at duty
  fr [duty]   front motor reverse at duty
  bf [duty]   BOTH motors forward (AWD)
  br [duty]   BOTH motors reverse (AWD)
  s           stop all motors
  c           center steering
  l [us]      steer left  by setting pulse-width (default us_min)
  r [us]      steer right by setting pulse-width (default us_max)
  u <us>      set servo to absolute pulse width
  sweep       sweep servo us_min ↔ us_max twice
  q           quit
""".strip()


def _parse_duty(arg: str | None, default: float = 0.25) -> float:
    if arg is None:
        return default
    try:
        return max(-1.0, min(1.0, float(arg)))
    except ValueError:
        return default


def main() -> int:
    cfg = config.load()
    motors = MotorHAT(cfg.drive, timeout_ms=cfg.safety.motor_timeout_ms)
    servo = SteeringServo(cfg.steering, cfg.geometry)
    print("Motor + servo ready. Type 'help' for commands. Wheels OFF the ground!")
    print(HELP)
    try:
        while True:
            try:
                line = input("> ").strip()
            except EOFError:
                break
            if not line:
                continue
            parts = line.split()
            cmd = parts[0].lower()
            arg = parts[1] if len(parts) > 1 else None

            if cmd in ("q", "quit", "exit"):
                break
            elif cmd == "help":
                print(HELP)
            elif cmd == "s":
                motors.stop()
            elif cmd == "rf":
                motors.set_each(rear=_parse_duty(arg), front=0.0)
            elif cmd == "rr":
                motors.set_each(rear=-_parse_duty(arg), front=0.0)
            elif cmd == "ff":
                motors.set_each(rear=0.0, front=_parse_duty(arg))
            elif cmd == "fr":
                motors.set_each(rear=0.0, front=-_parse_duty(arg))
            elif cmd == "bf":
                motors.set_throttle(_parse_duty(arg))
            elif cmd == "br":
                motors.set_throttle(-_parse_duty(arg))
            elif cmd == "c":
                servo.center()
            elif cmd == "l":
                us = int(arg) if arg else cfg.steering.us_min
                servo.set_us(us)
            elif cmd == "r":
                us = int(arg) if arg else cfg.steering.us_max
                servo.set_us(us)
            elif cmd == "u":
                if arg is None:
                    print("usage: u <us>")
                else:
                    servo.set_us(int(arg))
            elif cmd == "sweep":
                for _ in range(2):
                    for us in range(cfg.steering.us_min, cfg.steering.us_max + 1, 25):
                        servo.set_us(us)
                        time.sleep(0.02)
                    for us in range(cfg.steering.us_max, cfg.steering.us_min - 1, -25):
                        servo.set_us(us)
                        time.sleep(0.02)
                servo.center()
            else:
                print(f"unknown: {cmd}")
    finally:
        motors.close()
        servo.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
