"""Drive-to-map: record perimeter + keep-out loops by driving the rover.

Per brief §6:
  - Perimeter: drop a breadcrumb every `sample_m`; on Finish, connect last→first,
    Douglas-Peucker simplify, optionally inset.
  - Keep-out: drop breadcrumbs; once cumulative length > loop_min_perimeter_m
    AND current position is within loop_closure_m of the loop's start, the loop
    auto-closes and the enclosed area becomes a no-go polygon.
"""
from __future__ import annotations

import math
import threading
from dataclasses import dataclass, field
from pathlib import Path

import yaml

from ..config import TeachCfg
from .geo import Origin, to_ll
from .geometry import Polygon, douglas_peucker, is_simple, polyline_length, signed_area


Point = tuple[float, float]


@dataclass
class TeachState:
    mode: str = "idle"  # idle | perimeter | keepout
    label: str = ""
    track: list[Point] = field(default_factory=list)
    distance_to_close: float | None = None


class TeachRecorder:
    def __init__(self, cfg: TeachCfg, origin: Origin):
        self.cfg = cfg
        self.origin = origin
        self._lock = threading.Lock()
        self.state = TeachState()
        self.boundary: Polygon | None = None
        self.keepouts: list[tuple[str, Polygon]] = []

    # ---- public API -----------------------------------------------------

    def start_perimeter(self) -> None:
        with self._lock:
            self.state = TeachState(mode="perimeter")

    def start_keepout(self, label: str = "") -> None:
        with self._lock:
            self.state = TeachState(mode="keepout", label=label or f"zone-{len(self.keepouts)+1}")

    def sample(self, point: Point) -> dict:
        """Drop a breadcrumb if we've moved >= sample_m. Returns telemetry."""
        with self._lock:
            tr = self.state.track
            if not tr or math.hypot(point[0] - tr[-1][0], point[1] - tr[-1][1]) >= self.cfg.sample_m:
                tr.append(point)
            # Loop-closure hint for keep-out mode.
            self.state.distance_to_close = None
            if self.state.mode == "keepout" and len(tr) >= 3:
                length = polyline_length(tr)
                if length >= self.cfg.loop_min_perimeter_m:
                    d = math.hypot(tr[0][0] - point[0], tr[0][1] - point[1])
                    self.state.distance_to_close = d
                    if d <= self.cfg.loop_closure_m and self.cfg.auto_confirm_closure:
                        self._finalize_keepout_locked()
            return {
                "mode": self.state.mode,
                "label": self.state.label,
                "samples": len(tr),
                "distance_to_close": self.state.distance_to_close,
            }

    def undo(self) -> None:
        with self._lock:
            if self.state.track:
                self.state.track.pop()

    def finish(self) -> str:
        """Manually finalize the current track. Returns a status string."""
        with self._lock:
            if self.state.mode == "perimeter":
                return self._finalize_perimeter_locked()
            if self.state.mode == "keepout":
                return self._finalize_keepout_locked()
            return "not teaching"

    def cancel(self) -> None:
        with self._lock:
            self.state = TeachState()

    def delete_keepout(self, label: str) -> bool:
        with self._lock:
            n0 = len(self.keepouts)
            self.keepouts = [(l, p) for l, p in self.keepouts if l != label]
            return len(self.keepouts) < n0

    def save_yaml(self, path: str | Path) -> None:
        with self._lock:
            doc = {
                "origin": {"lat": self.origin.lat, "lon": self.origin.lon},
                "boundary": [
                    {"lat": ll[0], "lon": ll[1]}
                    for ll in (to_ll(p[0], p[1], self.origin) for p in (self.boundary or []))
                ],
                "keepouts": [
                    {
                        "type": "polygon",
                        "label": label,
                        "points": [
                            {"lat": ll[0], "lon": ll[1]}
                            for ll in (to_ll(p[0], p[1], self.origin) for p in poly)
                        ],
                    }
                    for label, poly in self.keepouts
                ],
                "inset_m": self.cfg.boundary_inset_m,
            }
        Path(path).write_text(yaml.safe_dump(doc, sort_keys=False))

    def snapshot(self) -> dict:
        with self._lock:
            return {
                "mode": self.state.mode,
                "label": self.state.label,
                "track": list(self.state.track),
                "distance_to_close": self.state.distance_to_close,
                "boundary": list(self.boundary) if self.boundary else None,
                "keepouts": [{"label": l, "points": list(p)} for l, p in self.keepouts],
            }

    # ---- internals ------------------------------------------------------

    def _finalize_perimeter_locked(self) -> str:
        tr = self.state.track
        if len(tr) < 4:
            return "perimeter too short"
        poly = douglas_peucker(tr, self.cfg.simplify_m)
        if poly[0] != poly[-1]:
            poly_closed = poly + [poly[0]]
        else:
            poly_closed = poly
        # Strip the closing duplicate before storing.
        ring = poly_closed[:-1]
        if not is_simple(ring):
            return "perimeter self-intersects — redrive"
        if abs(signed_area(ring)) < 1.0:
            return "perimeter encloses too little area"
        self.boundary = ring
        self.state = TeachState()
        return "perimeter saved"

    def _finalize_keepout_locked(self) -> str:
        tr = self.state.track
        if len(tr) < 4:
            return "loop too short"
        # Close the loop end → start.
        ring = douglas_peucker(tr + [tr[0]], self.cfg.simplify_m)[:-1]
        if not is_simple(ring):
            return "keep-out self-intersects — redrive"
        if abs(signed_area(ring)) < 0.25:
            return "keep-out too tiny"
        self.keepouts.append((self.state.label, ring))
        self.state = TeachState()
        return f"keep-out saved ({self.keepouts[-1][0]})"


def load_boundary_yaml(path: str | Path) -> dict:
    """Load `boundary.yaml`. Returns dict with origin, boundary, keepouts."""
    with open(path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f)
