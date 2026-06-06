"""PiSugar UPS client over its Unix domain socket (/tmp/pisugar-server.sock).

The PiSugar power-manager exposes a tiny line protocol: send a command,
receive one line. Commands we care about:
  - "get battery"                → "battery: 73.45"
  - "get battery_v"              → "battery_v: 4.02"   (Volts)
  - "get battery_i"              → "battery_i: -0.15"  (Amps, negative = draining)
  - "get battery_power_plugged"  → "battery_power_plugged: true"
  - "get battery_charging"       → "battery_charging: false"
  - "get model"                  → "model: PiSugar 3"

Safe shutdown ON CRITICAL is handled by pisugar-server itself via its
`safe_shutdown_level` config — we just need to read state for the safety
monitor and surface it on the UI.

Returns dataclass values rather than raw strings; falls back gracefully if
the socket isn't available so dev-box runs don't crash.
"""
from __future__ import annotations

import socket
import threading
import time
from dataclasses import dataclass


SOCK_PATH = "/tmp/pisugar-server.sock"


@dataclass
class BatteryState:
    percent: float = -1.0
    charging: bool = False
    plugged: bool = False
    voltage_v: float = 0.0
    current_a: float = 0.0          # negative = draining, positive = charging
    model: str = ""
    available: bool = False
    last_read_mono: float = 0.0


class PiSugar:
    def __init__(self, sock_path: str = SOCK_PATH, timeout_s: float = 0.5):
        self.sock_path = sock_path
        self.timeout_s = timeout_s
        self._lock = threading.Lock()
        self.state = BatteryState()

    def poll(self) -> BatteryState:
        """Block briefly to refresh battery details. Safe to call ~1 Hz."""
        try:
            pct = self._ask("get battery")
            volt = self._ask("get battery_v")
            curr = self._ask("get battery_i")
            chg = self._ask("get battery_charging")
            plug = self._ask("get battery_power_plugged")
            model = self._ask("get model")
            with self._lock:
                self.state.percent = _parse_float(pct)
                self.state.voltage_v = _parse_float(volt)
                self.state.current_a = _parse_float(curr)
                self.state.charging = "true" in chg.lower()
                self.state.plugged = "true" in plug.lower()
                self.state.model = model.split(":", 1)[-1].strip() if ":" in model else model.strip()
                self.state.available = True
                self.state.last_read_mono = time.monotonic()
        except Exception:
            with self._lock:
                self.state.available = False
        with self._lock:
            return BatteryState(**self.state.__dict__)

    def _ask(self, cmd: str) -> str:
        s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        s.settimeout(self.timeout_s)
        try:
            s.connect(self.sock_path)
            s.sendall((cmd + "\n").encode())
            data = s.recv(256).decode("ascii", errors="ignore").strip()
            return data
        finally:
            s.close()


def _parse_float(line: str) -> float:
    """Extract the float from a 'key: 12.3' line; returns 0.0 if it can't."""
    try:
        return float(line.split(":")[-1].strip())
    except (ValueError, AttributeError):
        return 0.0
