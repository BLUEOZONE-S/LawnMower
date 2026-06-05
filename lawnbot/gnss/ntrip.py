"""NTRIP client → stream RTCM3 corrections to the GPS UART.

Uses pygnssutils.GNSSNTRIPClient (the standard Python NTRIP client). Runs in
its own thread, hands incoming RTCM bytes to the LC29H by writing them
straight through the GPS's UART (per LC29H app note).

If the caster is unreachable, this silently retries with backoff. The
safety monitor's RTK-age trip is what turns "no corrections" into a stop —
the NTRIP client itself is best-effort.
"""
from __future__ import annotations

import threading
import time
from typing import Callable

try:
    from pygnssutils import GNSSNTRIPClient  # type: ignore
    _HAS_NTRIP = True
except ImportError:
    _HAS_NTRIP = False

from ..config import NtripCfg


class NtripForwarder:
    def __init__(self, cfg: NtripCfg, write_rtcm: Callable[[bytes], None]):
        self.cfg = cfg
        self._write = write_rtcm
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._client = None

    @property
    def enabled(self) -> bool:
        return bool(self.cfg.host) and bool(self.cfg.mountpoint)

    def start(self) -> None:
        if not self.enabled:
            return
        if not _HAS_NTRIP:
            raise RuntimeError("pygnssutils not installed")
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._client is not None:
            try:
                self._client.stop()
            except Exception:
                pass
        if self._thread:
            self._thread.join(timeout=2.0)

    def _run(self) -> None:
        backoff = 1.0
        while not self._stop.is_set():
            try:
                self._client = GNSSNTRIPClient()
                # Output queue: pygnssutils delivers RTCM messages here.
                import queue
                outq: queue.Queue = queue.Queue()
                self._client.run(
                    server=self.cfg.host,
                    port=self.cfg.port,
                    https=False,
                    mountpoint=self.cfg.mountpoint,
                    ntripuser=self.cfg.user,
                    ntrippassword=self.cfg.password,
                    output=outq,
                )
                while not self._stop.is_set():
                    try:
                        raw, _parsed = outq.get(timeout=1.0)
                    except Exception:
                        continue
                    if raw:
                        self._write(raw)
                backoff = 1.0
            except Exception:
                # On any failure, sleep then reconnect with backoff.
                time.sleep(min(30.0, backoff))
                backoff = min(30.0, backoff * 2)
