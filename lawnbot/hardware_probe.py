"""Live hardware-presence probe + host metrics for the UI status panel.

Cheap, infrequent introspection: scans the I2C bus for the project's expected
addresses, checks /dev/serial0 and the PiSugar socket, reads CPU/RAM/temp from
psutil + /sys/class/thermal. Caches results so the WebSocket push loop (10 Hz)
doesn't slam the I2C bus.
"""
from __future__ import annotations

import os
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path

try:
    from smbus2 import SMBus
except ImportError:
    SMBus = None  # type: ignore

try:
    import psutil
except ImportError:
    psutil = None  # type: ignore


# Project's expected I2C devices, in scan order.
I2C_DEVICES = [
    (0x40, "Motor HAT (PCA9685)"),
    (0x4A, "BNO085 IMU"),
    (0x57, "PiSugar 3 battery"),
    (0x68, "PiSugar 3 RTC"),
    (0x75, "PiSugar 2/S battery"),
    (0x32, "PiSugar 2/S RTC"),
]

PISUGAR_SOCK = "/tmp/pisugar-server.sock"


@dataclass
class I2CResult:
    addr: int
    label: str
    present: bool


@dataclass
class HardwareStatus:
    i2c_bus: int = 1
    i2c_available: bool = False
    devices: list[I2CResult] = field(default_factory=list)
    motor_hat: bool = False
    imu: bool = False
    pisugar: bool = False                # ANY pisugar address detected
    pisugar_socket: bool = False
    serial0_path: str = "/dev/serial0"
    serial0_present: bool = False
    serial0_target: str = ""             # readlink target
    serial0_bytes_total: int = 0         # cumulative bytes read by GPS reader (if exposed)
    gpiochip: str = ""
    gpiochip_accessible: bool = False
    last_scan_age_s: float = 0.0


@dataclass
class SystemStatus:
    cpu_pct: float = 0.0
    mem_pct: float = 0.0
    mem_used_mb: float = 0.0
    mem_total_mb: float = 0.0
    temp_c: float = 0.0
    uptime_s: float = 0.0
    load_1: float = 0.0


class HardwareProbe:
    """Lazy, throttled probe. Call :meth:`snapshot` from the UI push loop —
    real work runs at most once every ``scan_interval_s`` seconds.
    """

    def __init__(self, scan_interval_s: float = 2.0):
        self._scan_interval = scan_interval_s
        self._lock = threading.Lock()
        self._hw = HardwareStatus()
        self._sys = SystemStatus()
        self._last_scan = 0.0
        self._gps_reader = None  # set by Runtime if it wants byte-count exposure

        # Prime psutil — first call to cpu_percent() returns 0.0; we want real numbers.
        if psutil is not None:
            psutil.cpu_percent(interval=None)

    def attach_gps_reader(self, gps_reader) -> None:
        """Optional: tell the probe about the LC29H reader so it can surface
        the cumulative byte-count from the UART. The reader is duck-typed —
        it just needs ``.bytes_read`` (int) attribute or similar.
        """
        self._gps_reader = gps_reader

    def snapshot(self) -> dict:
        now = time.monotonic()
        with self._lock:
            if now - self._last_scan >= self._scan_interval:
                self._refresh(now)
            return {
                "hardware": {
                    "i2c_bus": self._hw.i2c_bus,
                    "i2c_available": self._hw.i2c_available,
                    "devices": [
                        {"addr": f"0x{d.addr:02X}", "label": d.label, "present": d.present}
                        for d in self._hw.devices
                    ],
                    "motor_hat": self._hw.motor_hat,
                    "imu": self._hw.imu,
                    "pisugar": self._hw.pisugar,
                    "pisugar_socket": self._hw.pisugar_socket,
                    "serial0": {
                        "path": self._hw.serial0_path,
                        "present": self._hw.serial0_present,
                        "target": self._hw.serial0_target,
                        "bytes_total": self._hw.serial0_bytes_total,
                    },
                    "gpiochip": {
                        "path": self._hw.gpiochip,
                        "accessible": self._hw.gpiochip_accessible,
                    },
                    "last_scan_age_s": round(now - self._last_scan, 2),
                },
                "system": {
                    "cpu_pct": round(self._sys.cpu_pct, 1),
                    "mem_pct": round(self._sys.mem_pct, 1),
                    "mem_used_mb": round(self._sys.mem_used_mb, 1),
                    "mem_total_mb": round(self._sys.mem_total_mb, 1),
                    "temp_c": round(self._sys.temp_c, 1),
                    "uptime_s": int(self._sys.uptime_s),
                    "load_1": round(self._sys.load_1, 2),
                },
            }

    # ------------------------------------------------------------------ refresh

    def _refresh(self, now: float) -> None:
        self._last_scan = now
        self._refresh_i2c()
        self._refresh_serial0()
        self._refresh_gpio()
        self._refresh_socket()
        self._refresh_system()

    def _refresh_i2c(self) -> None:
        self._hw.devices = []
        if SMBus is None:
            self._hw.i2c_available = False
            return
        try:
            bus = SMBus(self._hw.i2c_bus)
        except (FileNotFoundError, PermissionError, OSError):
            self._hw.i2c_available = False
            return
        self._hw.i2c_available = True
        try:
            for addr, label in I2C_DEVICES:
                present = False
                # A 0-byte quick-write is the standard "is anything home?" probe.
                # Falls back to read_byte for devices that NACK on quick.
                try:
                    bus.write_quick(addr)
                    present = True
                except OSError:
                    try:
                        bus.read_byte(addr)
                        present = True
                    except OSError:
                        present = False
                self._hw.devices.append(I2CResult(addr=addr, label=label, present=present))
        finally:
            try:
                bus.close()
            except Exception:
                pass

        present_addrs = {r.addr for r in self._hw.devices if r.present}
        self._hw.motor_hat = 0x40 in present_addrs
        self._hw.imu = 0x4A in present_addrs
        self._hw.pisugar = bool(present_addrs & {0x57, 0x68, 0x75, 0x32})

    def _refresh_serial0(self) -> None:
        self._hw.serial0_present = os.path.exists(self._hw.serial0_path)
        if self._hw.serial0_present:
            try:
                self._hw.serial0_target = os.path.realpath(self._hw.serial0_path)
            except OSError:
                self._hw.serial0_target = ""
        else:
            self._hw.serial0_target = ""
        if self._gps_reader is not None:
            self._hw.serial0_bytes_total = int(getattr(self._gps_reader, "bytes_read", 0) or 0)

    def _refresh_gpio(self) -> None:
        # Same auto-detect as servo.py — Pi 5 uses gpiochip4 (or 0 on Kali variants);
        # Pi 4 / Zero 2 W use gpiochip0.
        for n in (4, 0):
            path = f"/dev/gpiochip{n}"
            if os.path.exists(path):
                self._hw.gpiochip = path
                self._hw.gpiochip_accessible = os.access(path, os.R_OK | os.W_OK)
                return
        self._hw.gpiochip = ""
        self._hw.gpiochip_accessible = False

    def _refresh_socket(self) -> None:
        self._hw.pisugar_socket = Path(PISUGAR_SOCK).is_socket() if hasattr(Path(PISUGAR_SOCK), "is_socket") else os.path.exists(PISUGAR_SOCK)

    def _refresh_system(self) -> None:
        if psutil is not None:
            self._sys.cpu_pct = psutil.cpu_percent(interval=None)
            vm = psutil.virtual_memory()
            self._sys.mem_pct = vm.percent
            self._sys.mem_used_mb = vm.used / (1024 * 1024)
            self._sys.mem_total_mb = vm.total / (1024 * 1024)
            try:
                self._sys.uptime_s = time.time() - psutil.boot_time()
            except Exception:
                self._sys.uptime_s = 0.0
            try:
                self._sys.load_1 = os.getloadavg()[0]
            except (AttributeError, OSError):
                self._sys.load_1 = 0.0

        # CPU temp via thermal_zone0 (Pi). Falls back to psutil if available.
        self._sys.temp_c = 0.0
        try:
            with open("/sys/class/thermal/thermal_zone0/temp", "r") as f:
                self._sys.temp_c = int(f.read().strip()) / 1000.0
        except (FileNotFoundError, PermissionError, ValueError):
            if psutil is not None:
                try:
                    sensors = psutil.sensors_temperatures()
                    for _name, entries in (sensors or {}).items():
                        if entries:
                            self._sys.temp_c = entries[0].current
                            break
                except (AttributeError, OSError):
                    pass
