"""LC29H(DA) GNSS reader.

Opens the UART, reads NMEA, and exposes the latest fix. The DA variant is
RTK-only at 1 Hz with no on-chip dead reckoning, so the estimator carries
the rover between fixes using IMU yaw + wheel odometry.

The reader runs in its own thread; the control loop never blocks on the
serial port. GGA gives lat/lon/quality/sats/HDOP; GSV gives per-satellite
PRN/elevation/azimuth/SNR for the debug panel. RMC/VTG can be added later
if a heading-of-motion fallback is wanted.
"""
from __future__ import annotations

import threading
import time
from dataclasses import dataclass, field

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


# NMEA talker IDs → human-readable constellation labels.
CONSTELLATION = {
    "GP": "GPS",
    "GL": "GLONASS",
    "GA": "Galileo",
    "GB": "BeiDou",
    "BD": "BeiDou",
    "GQ": "QZSS",
    "QZ": "QZSS",
    "GI": "IRNSS",
    "GN": "Mixed",
    "SB": "SBAS",
}


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


@dataclass
class SatelliteInfo:
    """One satellite's instantaneous geometry + signal strength."""

    prn: int                    # PRN number (1..32 GPS, etc.)
    talker: str                 # NMEA talker id, e.g. "GP", "GL", "GA"
    constellation: str          # Human-readable, e.g. "GPS"
    elevation_deg: float        # 0..90 (0 = horizon, 90 = zenith)
    azimuth_deg: float          # 0..360 (clockwise from true north)
    snr_dbhz: float             # signal-to-noise, ~0..55 (0 = not tracked)
    used: bool = False          # used in the current position fix


@dataclass
class SatelliteSnapshot:
    """Aggregated view across every constellation reported by the receiver."""

    sats: list[SatelliteInfo] = field(default_factory=list)
    timestamp_mono: float = 0.0

    @property
    def age_s(self) -> float:
        return time.monotonic() - self.timestamp_mono if self.timestamp_mono else float("inf")


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
        self._gsv_count = 0
        # GSV sentences arrive in groups (one talker = one group across several
        # sentences). Accumulate into a staging dict keyed by talker, then
        # publish once the group is complete so the panel sees consistent data.
        self._gsv_partial: dict[str, dict[int, SatelliteInfo]] = {}
        self._gsv_seen_groups: dict[str, int] = {}
        self._gsv_total_groups: dict[str, int] = {}
        self._sats_by_talker: dict[str, list[SatelliteInfo]] = {}
        self._used_prns: set[tuple[str, int]] = set()
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    @property
    def latest(self) -> GnssFix | None:
        with self._lock:
            return self._latest

    @property
    def satellites(self) -> SatelliteSnapshot:
        """Per-satellite skyplot data, aggregated across all constellations."""
        with self._lock:
            all_sats: list[SatelliteInfo] = []
            for talker, sats in self._sats_by_talker.items():
                for s in sats:
                    s.used = (talker, s.prn) in self._used_prns
                    all_sats.append(s)
            return SatelliteSnapshot(sats=all_sats, timestamp_mono=time.monotonic())

    @property
    def stats(self) -> dict:
        with self._lock:
            return {
                "raw_lines": self._raw_count,
                "gga_lines": self._gga_count,
                "gsv_lines": self._gsv_count,
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
            if not text.startswith("$"):
                continue
            # Talker id is the two chars after the '$' for the standard $TTGGA form.
            talker = text[1:3].upper()
            sentence = text[3:6].upper()

            if sentence == "GGA":
                self._handle_gga(text)
            elif sentence == "GSV":
                self._handle_gsv(text, talker)
            elif sentence == "GSA":
                self._handle_gsa(text, talker)

    def _handle_gga(self, text: str) -> None:
        try:
            msg = pynmea2.parse(text)
        except Exception:
            return
        if msg.latitude is None or msg.longitude is None:
            return
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
            return
        with self._lock:
            self._latest = fix
            self._gga_count += 1

    def _handle_gsv(self, text: str, talker: str) -> None:
        try:
            msg = pynmea2.parse(text)
        except Exception:
            return
        # pynmea2 exposes sv_prn_num_1..4, elevation_deg_1..4, azimuth_1..4, snr_1..4.
        try:
            total_groups = int(msg.num_messages)
            group_idx = int(msg.msg_num)
        except (AttributeError, TypeError, ValueError):
            return
        with self._lock:
            self._gsv_count += 1
            # Reset accumulator at the start of a new group cycle.
            if group_idx == 1:
                self._gsv_partial[talker] = {}
            partial = self._gsv_partial.setdefault(talker, {})
            for slot in (1, 2, 3, 4):
                prn_attr = getattr(msg, f"sv_prn_num_{slot}", None)
                if not prn_attr:
                    continue
                try:
                    prn = int(prn_attr)
                except (TypeError, ValueError):
                    continue
                el = _safe_float(getattr(msg, f"elevation_deg_{slot}", None))
                az = _safe_float(getattr(msg, f"azimuth_{slot}", None))
                snr = _safe_float(getattr(msg, f"snr_{slot}", None))
                partial[prn] = SatelliteInfo(
                    prn=prn,
                    talker=talker,
                    constellation=CONSTELLATION.get(talker, talker),
                    elevation_deg=el or 0.0,
                    azimuth_deg=az or 0.0,
                    snr_dbhz=snr or 0.0,
                )
            self._gsv_total_groups[talker] = total_groups
            self._gsv_seen_groups[talker] = group_idx
            if group_idx >= total_groups:
                # Group complete — publish.
                self._sats_by_talker[talker] = list(partial.values())

    def _handle_gsa(self, text: str, talker: str) -> None:
        """GSA carries the list of PRNs used in the position fix."""
        try:
            msg = pynmea2.parse(text)
        except Exception:
            return
        used: set[tuple[str, int]] = set()
        for i in range(1, 13):
            attr = getattr(msg, f"sv_id{i:02d}", None)
            if not attr:
                continue
            try:
                used.add((talker, int(attr)))
            except (TypeError, ValueError):
                continue
        with self._lock:
            # Drop stale entries from this talker, then merge in fresh.
            self._used_prns = {p for p in self._used_prns if p[0] != talker}
            self._used_prns |= used


def _safe_float(v) -> float | None:
    if v is None or v == "":
        return None
    try:
        return float(v)
    except (TypeError, ValueError):
        return None
