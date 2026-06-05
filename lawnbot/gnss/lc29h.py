"""LC29H(DA) GNSS reader.

Opens the UART, reads NMEA, and exposes the latest fix. The DA variant is
RTK-only at 1 Hz with no on-chip dead reckoning, so the estimator carries
the rover between fixes using IMU yaw + wheel odometry.

The reader runs in its own thread; the control loop never blocks on the
serial port. Only GGA is parsed (lat, lon, fix quality, sats, HDOP) — that's
all the planner needs. RMC/VTG can be added later if a heading-of-motion
fallback is wanted.
"""
from __future__ import annotations

import threading
import time
from dataclasses import dataclass

try:
    import serial
except ImportError:
    serial = None  # type: ignore

try:
    import pynmea2
except ImportError:
    pynmea2 = None  # type: ignore

from ..config import GnssCfg


# NMEA GGA fix-quality codes
FIX_INVALID = 0
FIX_SINGLE = 1
FIX_DGPS = 2
FIX_PPS = 3
FIX_RTK_FIXED = 4
FIX_RTK_FLOAT = 5


@dataclass
class GnssFix:
    lat: float
    lon: float
    quality: int
    sats: int
    hdop: float
    alt_m: float
    timestamp_mono: float  # time.monotonic() when received

    @property
    def age_s(self) -> float:
        return time.monotonic() - self.timestamp_mono


class LC29H:
    def __init__(self, cfg: GnssCfg):
        if serial is None or pynmea2 is None:
            raise RuntimeError("pyserial / pynmea2 not available — install on the Pi")
        self.cfg = cfg
        self._ser = serial.Serial(cfg.port, cfg.baud, timeout=1.0)
        self._lock = threading.Lock()
        self._latest: GnssFix | None = None
        self._raw_count = 0
        self._gga_count = 0
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    @property
    def latest(self) -> GnssFix | None:
        with self._lock:
            return self._latest

    @property
    def stats(self) -> dict:
        with self._lock:
            return {
                "raw_lines": self._raw_count,
                "gga_lines": self._gga_count,
                "last_quality": self._latest.quality if self._latest else None,
                "last_age_s": self._latest.age_s if self._latest else None,
            }

    def write_rtcm(self, data: bytes) -> None:
        """Forward RTCM3 bytes from NTRIP straight to the receiver UART."""
        self._ser.write(data)

    def close(self) -> None:
        self._stop.set()
        self._thread.join(timeout=2.0)
        try:
            self._ser.close()
        except Exception:
            pass

    # ---- internals ------------------------------------------------------

    def _run(self) -> None:
        while not self._stop.is_set():
            try:
                line = self._ser.readline()
            except Exception:
                time.sleep(0.1)
                continue
            if not line:
                continue
            with self._lock:
                self._raw_count += 1
            try:
                text = line.decode("ascii", errors="ignore").strip()
            except Exception:
                continue
            if not text.startswith("$") or "GGA" not in text:
                continue
            try:
                msg = pynmea2.parse(text)
            except Exception:
                continue
            if msg.latitude is None or msg.longitude is None:
                continue
            try:
                fix = GnssFix(
                    lat=float(msg.latitude),
                    lon=float(msg.longitude),
                    quality=int(msg.gps_qual or 0),
                    sats=int(msg.num_sats or 0),
                    hdop=float(msg.horizontal_dil or 0.0),
                    alt_m=float(msg.altitude or 0.0),
                    timestamp_mono=time.monotonic(),
                )
            except (TypeError, ValueError):
                continue
            with self._lock:
                self._latest = fix
                self._gga_count += 1
