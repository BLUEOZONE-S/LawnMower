"""CLI teach fallback. Prefer the GUI teach screen (§6) — this is the manual path.

Steps (run on the Pi while the rover has a stable RTK fix):
  1. python -m tools.teach_boundary perimeter         # drive the rover, press Enter at each corner
  2. python -m tools.teach_boundary keepout <label>   # drive a loop around an obstacle
  3. python -m tools.teach_boundary save              # writes boundary.yaml

Inputs are read live from the LC29H — no manual lat/lon typing.
"""
from __future__ import annotations

import sys
import time
from pathlib import Path

from lawnbot import config
from lawnbot.gnss.lc29h import LC29H
from lawnbot.nav.geo import Origin, to_enu
from lawnbot.nav.teach import TeachRecorder, load_boundary_yaml


def _wait_for_fix(gps: LC29H) -> tuple[float, float]:
    while True:
        fix = gps.latest
        if fix is not None and fix.quality >= 1:
            return fix.lat, fix.lon
        print("waiting for fix…", end="\r", flush=True)
        time.sleep(0.5)


def _ingest_until_enter(recorder: TeachRecorder, gps: LC29H) -> None:
    print("Drive. Press Enter to drop a checkpoint, type 'done' + Enter to finish.")
    while True:
        line = input(">> ").strip().lower()
        fix = gps.latest
        if fix is None:
            print("no fix")
            continue
        pt = to_enu(fix.lat, fix.lon, recorder.origin)
        info = recorder.sample(pt)
        if info["distance_to_close"] is not None:
            print(f"  {info['distance_to_close']:.2f} m to close")
        if line == "done":
            break


def main(argv: list[str]) -> int:
    cfg = config.load()
    gps = LC29H(cfg.gnss)
    try:
        # Get an origin: either from existing boundary.yaml or from the current fix.
        path = Path("boundary.yaml")
        if path.exists():
            doc = load_boundary_yaml(path)
            origin = Origin(lat=doc["origin"]["lat"], lon=doc["origin"]["lon"])
        else:
            lat, lon = _wait_for_fix(gps)
            origin = Origin(lat=lat, lon=lon)
            print(f"origin set to {lat:.7f}, {lon:.7f}")

        rec = TeachRecorder(cfg.teach, origin)

        cmd = argv[1] if len(argv) > 1 else "perimeter"
        if cmd == "perimeter":
            rec.start_perimeter()
            _ingest_until_enter(rec, gps)
            print(rec.finish())
        elif cmd == "keepout":
            label = argv[2] if len(argv) > 2 else ""
            rec.start_keepout(label)
            _ingest_until_enter(rec, gps)
            print(rec.finish())
        elif cmd == "save":
            rec.save_yaml("boundary.yaml")
            print("saved boundary.yaml")
        else:
            print(__doc__)
            return 1
    finally:
        gps.close()
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
