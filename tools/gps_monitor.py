"""Phase-2 bring-up: confirm the LC29H is producing fixes.

Prints the latest fix once per second. Run on the Pi after wiring the GPS
to /dev/serial0 and disabling the serial login console (see brief §3.1).

Usage:
    python3 -m tools.gps_monitor
"""
from __future__ import annotations

import sys
import time

from lawnbot import config
from lawnbot.gnss.lc29h import LC29H

_QUALITY = {
    0: "invalid",
    1: "single",
    2: "DGPS",
    3: "PPS",
    4: "RTK-fixed",
    5: "RTK-float",
}


def main() -> int:
    cfg = config.load()
    gps = LC29H(cfg.gnss)
    print(f"Listening on {cfg.gnss.port} @ {cfg.gnss.baud}. Ctrl-C to quit.")
    try:
        while True:
            fix = gps.latest
            stats = gps.stats
            if fix is None:
                print(
                    f"[no fix yet]  raw_lines={stats['raw_lines']:5d}  "
                    f"gga_lines={stats['gga_lines']:5d}"
                )
            else:
                q = _QUALITY.get(fix.quality, f"q{fix.quality}")
                print(
                    f"{fix.lat:11.7f}, {fix.lon:11.7f}  "
                    f"qual={q:<10}  sats={fix.sats:2d}  "
                    f"hdop={fix.hdop:4.2f}  age={fix.age_s:4.2f}s"
                )
            time.sleep(1.0)
    except KeyboardInterrupt:
        pass
    finally:
        gps.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
