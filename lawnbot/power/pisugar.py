"""PiSugar UPS client over its Unix domain socket (/tmp/pisugar-server.sock).

The PiSugar power-manager exposes a tiny line protocol: send a command,
receive one line. Two we care about:
  - "get battery"           → "battery: 73.45"
  - "get battery_power_plugged" → "battery_power_plugged: true"

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
    available: bool = False
    last_read_mono: float = 0.0


class PiSugar:
    def __init__(self, sock_path: str = SOCK_PATH, timeout_s: float = 0.5):
        self.sock_path = sock_path
        self.timeout_s = timeout_s
        self._lock = threading.Lock()
        self.state = BatteryState()

    def poll(self) -> BatteryState:
        """Block briefly to refresh percent + charging. Safe to call ~1 Hz."""
        try:
            pct = self._ask("get battery")
            chg = self._ask("get battery_power_plugged")
            with self._lock:
                self.state.percent = float(pct.split(":")[-1].strip())
                self.state.charging = "true" in chg.lower()
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
