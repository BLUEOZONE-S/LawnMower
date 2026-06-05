"""Lat/lon ↔ local ENU (East-North-Up) tangent plane.

The planner works in flat meters; GPS gives lat/lon. We pick a reference
origin (lat0, lon0) — the boundary centroid — and project everything
through a simple equirectangular tangent plane. Fine for any single yard
(<~200 m). For larger sites swap to pyproj-UTM; the API stays the same.
"""
from __future__ import annotations

import math
from dataclasses import dataclass


_LAT_METERS_PER_DEG = 110540.0  # near 45° lat; close enough for a yard
_LON_METERS_PER_DEG = 111320.0  # times cos(lat0)


@dataclass(frozen=True)
class Origin:
    lat: float
    lon: float

    @property
    def lon_scale(self) -> float:
        return _LON_METERS_PER_DEG * math.cos(math.radians(self.lat))


def to_enu(lat: float, lon: float, origin: Origin) -> tuple[float, float]:
    east = (lon - origin.lon) * origin.lon_scale
    north = (lat - origin.lat) * _LAT_METERS_PER_DEG
    return east, north


def to_ll(east: float, north: float, origin: Origin) -> tuple[float, float]:
    lon = origin.lon + east / origin.lon_scale
    lat = origin.lat + north / _LAT_METERS_PER_DEG
    return lat, lon


def centroid_ll(points_ll: list[tuple[float, float]]) -> Origin:
    """Average lat/lon — good enough as an ENU reference for a yard."""
    n = len(points_ll)
    if n == 0:
        raise ValueError("empty points")
    lat = sum(p[0] for p in points_ll) / n
    lon = sum(p[1] for p in points_ll) / n
    return Origin(lat=lat, lon=lon)
