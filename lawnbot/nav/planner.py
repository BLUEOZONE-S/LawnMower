"""Coverage planner — boustrophedon stripes + A* lawn-only connectors + crosscut.

Mirrors the validated sim:
  1. Build a drivable mask = lawn - keep-outs - body clearance.
  2. Generate stripes along the primary axis at deck-width spacing.
  3. Per stripe, scan-line the boundary, subtract keep-out projections,
     serpentine the segments.
  4. Connect consecutive cut-points: straight line if LOS clear; otherwise
     A* on the drivable grid with string-pull simplification.
  5. Run a second pass perpendicular to the first (crosscut). Connect.
  6. Return one flat waypoint list. Runs once per mission and caches.
"""
from __future__ import annotations

import heapq
import math
from dataclasses import dataclass

from .geometry import (
    DrivableMask,
    Polygon,
    bbox,
    dist,
    line_clips_polygon_y,
    los_clear,
    point_in_polygon,
    polygon_centroid,
    rotate_point,
    rotate_polygon,
    subtract_intervals,
)


Point = tuple[float, float]


@dataclass(frozen=True)
class PlanParams:
    deck_m: float
    body_clearance_m: float
    keepout_inflate_m: float
    crosscut: bool
    grid_cell_m: float = 0.2
    # Extra inset at each stripe end, on top of body_clearance_m. Set this to
    # ~R_min for Ackermann platforms so the U-turn between rows fits without
    # crossing the boundary. Zero for diff-drive (pivot-in-place is free).
    headland_m: float = 0.0
    # Stripe overlap as a fraction of deck_m. 0 = stripes touch, 0.1 = 10%
    # overlap (effective spacing = 0.9·deck_m). Clamped to [0, 0.5].
    overlap_pct: float = 0.0
    # Direction of the primary stripe pass: "h" = east-west, "v" = north-south.
    # The crosscut (if enabled) runs perpendicular to this. Boustrophedon only.
    primary_axis: str = "h"
    # Coverage pattern preset. See PATTERN_NAMES for the full list.
    pattern: str = "boustrophedon"


# Patterns expressed as a list of stripe-pass angles (radians off +x axis).
# A "pass" is one boustrophedon-style scan in the rotated frame; multiple
# passes overlay to produce richer coverage at the cost of more waypoints.
PATTERN_ANGLES: dict[str, list[float]] = {
    "boustrophedon": [0.0],
    "crosshatch":    [0.0, math.pi / 2],
    "diamond":       [math.pi / 4, 3 * math.pi / 4],
    "triangle":      [0.0, math.pi / 3, 2 * math.pi / 3],
    "star":          [k * math.pi / 6 for k in range(6)],  # 0°, 30°, …, 150°
}

PATTERN_NAMES = list(PATTERN_ANGLES.keys()) + ["spiral", "wave"]


def _stripes(
    boundary: Polygon,
    keepouts: list[Polygon],
    deck_m: float,
    inflate_m: float,
    axis: str,
    end_inset_m: float | None = None,
) -> list[list[Point]]:
    """Return a serpentine list of segments along the chosen axis.

    ``inflate_m`` controls the keep-out projection margin. ``end_inset_m`` (if
    provided) overrides how much each stripe is trimmed at the boundary ends —
    use this to leave headland space for an Ackermann U-turn. Defaults to
    ``inflate_m`` (legacy behavior).
    """
    if end_inset_m is None:
        end_inset_m = inflate_m
    x0, y0, x1, y1 = bbox(boundary)
    if axis == "h":
        v0, v1 = y0, y1
        line_clip = lambda v: line_clips_polygon_y(v, boundary)  # noqa: E731
        ko_proj = lambda ko, v: line_clips_polygon_y(v, ko)  # noqa: E731
        make_seg = lambda a, b, v: [(a, v), (b, v)]  # noqa: E731
    else:
        v0, v1 = x0, x1
        rot = lambda poly: [(y, x) for x, y in poly]  # noqa: E731
        boundary_r = rot(boundary)
        keepouts_r = [rot(k) for k in keepouts]
        line_clip = lambda v: line_clips_polygon_y(v, boundary_r)  # noqa: E731
        ko_proj = lambda ko, v: line_clips_polygon_y(v, ko)  # noqa: E731
        keepouts = keepouts_r
        make_seg = lambda a, b, v: [(v, a), (v, b)]  # noqa: E731

    segments: list[list[Point]] = []
    direction = +1
    v = v0 + deck_m / 2
    while v <= v1:
        # Inside-boundary x-intervals at this y.
        ivals = line_clip(v)
        # Carve keep-out intervals (inflated by body_clearance + ½ deck + a small margin).
        margin = inflate_m + deck_m / 2 + 0.05
        cuts: list[tuple[float, float]] = []
        for ko in keepouts:
            for c0, c1 in ko_proj(ko, v):
                cuts.append((c0 - margin, c1 + margin))
        ivals = subtract_intervals(ivals, cuts)
        # Trim each interval by end_inset_m at each end. The inset combines
        # boundary body-clearance with optional Ackermann headland room.
        ivals = [
            (a + end_inset_m, b - end_inset_m)
            for a, b in ivals
            if (b - end_inset_m) > (a + end_inset_m)
        ]
        if direction < 0:
            ivals = [(b, a) for a, b in reversed(ivals)]
        for a, b in ivals:
            segments.append(make_seg(a, b, v))
        direction = -direction
        v += deck_m
    return segments


def _astar(
    a: Point, b: Point, drivable: DrivableMask, allow_diagonals: bool = True
) -> list[Point] | None:
    """A* on the drivable grid from a → b. Returns world-space polyline or None."""
    cell = drivable.cell

    def world_to_ij(p: Point) -> tuple[int, int]:
        return int(round((p[0] - drivable.x0) / cell)), int(round((p[1] - drivable.y0) / cell))

    ai, aj = world_to_ij(a)
    bi, bj = world_to_ij(b)
    if not (0 <= ai < drivable.nx and 0 <= aj < drivable.ny):
        return None
    if not (0 <= bi < drivable.nx and 0 <= bj < drivable.ny):
        return None
    if not drivable._grid[aj][ai] or not drivable._grid[bj][bi]:
        return None

    if allow_diagonals:
        neighbors = [(-1, 0), (1, 0), (0, -1), (0, 1), (-1, -1), (1, -1), (-1, 1), (1, 1)]
    else:
        neighbors = [(-1, 0), (1, 0), (0, -1), (0, 1)]

    def h(i: int, j: int) -> float:
        di, dj = abs(i - bi), abs(j - bj)
        return math.hypot(di, dj) * cell

    start = (ai, aj)
    goal = (bi, bj)
    open_q: list[tuple[float, tuple[int, int]]] = []
    heapq.heappush(open_q, (0.0, start))
    came: dict[tuple[int, int], tuple[int, int]] = {}
    g_score: dict[tuple[int, int], float] = {start: 0.0}
    closed: set[tuple[int, int]] = set()

    while open_q:
        _, cur = heapq.heappop(open_q)
        if cur == goal:
            path_ij = [cur]
            while cur in came:
                cur = came[cur]
                path_ij.append(cur)
            path_ij.reverse()
            return [drivable.ij_to_xy(i, j) for i, j in path_ij]
        if cur in closed:
            continue
        closed.add(cur)
        ci, cj = cur
        for di, dj in neighbors:
            ni, nj = ci + di, cj + dj
            if not (0 <= ni < drivable.nx and 0 <= nj < drivable.ny):
                continue
            if not drivable._grid[nj][ni]:
                continue
            step = math.hypot(di, dj) * cell
            tentative = g_score[cur] + step
            if tentative < g_score.get((ni, nj), math.inf):
                came[(ni, nj)] = cur
                g_score[(ni, nj)] = tentative
                heapq.heappush(open_q, (tentative + h(ni, nj), (ni, nj)))
    return None


def _string_pull(path: list[Point], drivable: DrivableMask) -> list[Point]:
    """Drop intermediate vertices when the segment ahead is LOS-clear."""
    if len(path) <= 2:
        return list(path)
    out = [path[0]]
    i = 0
    while i < len(path) - 1:
        j = len(path) - 1
        while j > i + 1 and not los_clear(path[i], path[j], drivable):
            j -= 1
        out.append(path[j])
        i = j
    return out


def _connect(a: Point, b: Point, drivable: DrivableMask) -> list[Point]:
    """LOS straight, else A* + string-pull. Returns segment INCLUDING endpoints."""
    if los_clear(a, b, drivable):
        return [a, b]
    path = _astar(a, b, drivable)
    if path is None:
        return [a, b]  # fallback — let the controller try
    return _string_pull(path, drivable)


def _stripes_at_angle(
    boundary: Polygon,
    keepouts: list[Polygon],
    spacing: float,
    inflate: float,
    angle_rad: float,
    end_inset: float,
    origin: Point,
) -> list[list[Point]]:
    """Generate boustrophedon stripes oriented along ``angle_rad`` off +x.

    Implementation: rotate everything into a frame where stripes are
    horizontal, run the regular scan-line carver, then rotate the resulting
    waypoints back to ENU.
    """
    if abs(angle_rad) < 1e-9:
        return _stripes(boundary, keepouts, spacing, inflate, "h", end_inset_m=end_inset)
    if abs(angle_rad - math.pi / 2) < 1e-9:
        return _stripes(boundary, keepouts, spacing, inflate, "v", end_inset_m=end_inset)
    boundary_r = rotate_polygon(boundary, -angle_rad, origin)
    keepouts_r = [rotate_polygon(k, -angle_rad, origin) for k in keepouts]
    raw = _stripes(boundary_r, keepouts_r, spacing, inflate, "h", end_inset_m=end_inset)
    # Rotate every segment endpoint back into ENU.
    return [
        [rotate_point(p, angle_rad, origin) for p in seg]
        for seg in raw
    ]


def _spiral(
    boundary: Polygon,
    keepouts: list[Polygon],
    deck_m: float,
    drivable: DrivableMask,
) -> list[Point]:
    """Archimedean spiral seeded at the boundary centroid.

    r(θ) = (deck / 2π) · θ — one revolution every ``deck`` meters of radius.
    The point step is adapted so arc length stays ≈ deck/3 (gives the
    pure-pursuit controller a smooth, dense path to follow).
    """
    if deck_m <= 0:
        return []
    cx, cy = polygon_centroid(boundary)
    x0, y0, x1, y1 = bbox(boundary)
    max_r = math.hypot(x1 - x0, y1 - y0)  # cap so we don't run away
    b = deck_m / (2 * math.pi)
    waypoints: list[Point] = []
    theta = 0.0
    while True:
        r = b * theta
        if r > max_r:
            break
        x = cx + r * math.cos(theta)
        y = cy + r * math.sin(theta)
        if drivable.at(x, y):
            waypoints.append((x, y))
        # Adaptive step: keep arc-length step ≈ deck/3.
        d_theta = (deck_m / 3.0) / max(0.05, r)
        theta += d_theta
        if theta > 200 * math.pi:  # safety stop after 100 revolutions
            break
    return waypoints


def _wave_stripes(
    boundary: Polygon,
    keepouts: list[Polygon],
    spacing: float,
    inflate: float,
    end_inset: float,
    axis: str,
    amplitude_m: float,
    wavelength_m: float,
    drivable: DrivableMask,
) -> list[list[Point]]:
    """Boustrophedon stripes warped by a sinusoid perpendicular to travel.

    Each straight segment from ``_stripes`` is replaced with a wavy polyline.
    Wave amplitude is bounded so adjacent stripes don't cross.
    """
    raw = _stripes(boundary, keepouts, spacing, inflate, axis, end_inset_m=end_inset)
    amp = max(0.0, min(amplitude_m, 0.45 * spacing))
    if amp <= 1e-3 or wavelength_m <= 1e-3:
        return raw
    out: list[list[Point]] = []
    for seg in raw:
        a, b = seg[0], seg[1]
        if axis == "h":
            x0, x1 = a[0], b[0]
            y = a[1]
            length = abs(x1 - x0)
            steps = max(2, int(length / (wavelength_m * 0.1)))
            sgn = 1 if x1 >= x0 else -1
            pts = []
            for k in range(steps + 1):
                t = k / steps
                x = x0 + sgn * length * t
                phase = 2 * math.pi * (sgn * length * t) / wavelength_m
                yy = y + amp * math.sin(phase)
                if drivable.at(x, yy):
                    pts.append((x, yy))
            if len(pts) >= 2:
                out.append(pts)
        else:  # axis == "v"
            x = a[0]
            y0, y1 = a[1], b[1]
            length = abs(y1 - y0)
            steps = max(2, int(length / (wavelength_m * 0.1)))
            sgn = 1 if y1 >= y0 else -1
            pts = []
            for k in range(steps + 1):
                t = k / steps
                y = y0 + sgn * length * t
                phase = 2 * math.pi * (sgn * length * t) / wavelength_m
                xx = x + amp * math.sin(phase)
                if drivable.at(xx, y):
                    pts.append((xx, y))
            if len(pts) >= 2:
                out.append(pts)
    return out


def _stitch_segments(
    segments: list[list[Point]],
    drivable: DrivableMask,
) -> list[Point]:
    """Connect a serpentine list of segments into one waypoint stream.

    Each segment is itself a polyline (≥2 points). Between segments, drop a
    straight LOS or A* connector through drivable terrain.
    """
    waypoints: list[Point] = []
    last: Point | None = None
    for seg in segments:
        if not seg:
            continue
        if last is not None and last != seg[0]:
            connector = _connect(last, seg[0], drivable)
            waypoints.extend(connector[1:])
        else:
            waypoints.append(seg[0])
        for p in seg[1:]:
            waypoints.append(p)
        last = seg[-1]
    return waypoints


def _dedupe(waypoints: list[Point], min_step_m: float = 0.05) -> list[Point]:
    deduped: list[Point] = []
    for p in waypoints:
        if not deduped or dist(deduped[-1], p) > min_step_m:
            deduped.append(p)
    return deduped


def plan_coverage(
    boundary: Polygon,
    keepouts: list[Polygon],
    params: PlanParams,
) -> list[Point]:
    """Build the full waypoint list for the mission.

    Dispatches on ``params.pattern``:
      - boustrophedon / crosshatch / diamond / triangle / star → multi-angle
        stripe passes (each rotated to a different orientation).
      - spiral → Archimedean spiral from the boundary centroid outward.
      - wave   → sinusoidal stripes (one pass along ``primary_axis``).
    """
    inflate = params.body_clearance_m
    drivable = DrivableMask(boundary, keepouts, cell_m=params.grid_cell_m, inflate_m=inflate)
    end_inset = inflate + max(0.0, params.headland_m)
    overlap = max(0.0, min(0.5, float(params.overlap_pct)))
    spacing = max(0.05, params.deck_m * (1.0 - overlap))
    pattern = (params.pattern or "boustrophedon").lower()

    if pattern == "spiral":
        pts = _spiral(boundary, keepouts, spacing, drivable)
        return _dedupe(pts)

    if pattern == "wave":
        axis = params.primary_axis if params.primary_axis in ("h", "v") else "h"
        segments = _wave_stripes(
            boundary, keepouts, spacing, inflate, end_inset, axis,
            amplitude_m=spacing * 0.25,
            wavelength_m=max(spacing * 2.5, 1.0),
            drivable=drivable,
        )
        return _dedupe(_stitch_segments(segments, drivable))

    if pattern in PATTERN_ANGLES:
        origin = polygon_centroid(boundary)
        angles = list(PATTERN_ANGLES[pattern])
        # Honor primary_axis on the basic boustrophedon-style patterns by
        # rotating the whole pattern 90° if the user picked NS instead of EW.
        if pattern in ("boustrophedon",) and params.primary_axis == "v":
            angles = [a + math.pi / 2 for a in angles]
        # Boustrophedon's "crosscut" toggle adds a perpendicular pass — useful
        # for the legacy single-axis pattern. The other presets already encode
        # their own multi-pass structure so ignore the flag there.
        if pattern == "boustrophedon" and params.crosscut:
            angles.append(angles[0] + math.pi / 2)

        waypoints: list[Point] = []
        last: Point | None = None
        for angle in angles:
            segs = _stripes_at_angle(boundary, keepouts, spacing, inflate, angle, end_inset, origin)
            for seg in segs:
                if last is not None and last != seg[0]:
                    connector = _connect(last, seg[0], drivable)
                    waypoints.extend(connector[1:])
                else:
                    waypoints.append(seg[0])
                for p in seg[1:]:
                    waypoints.append(p)
                last = seg[-1]
        return _dedupe(waypoints)

    # Unknown pattern → safe fallback.
    return _dedupe(plan_coverage(
        boundary, keepouts,
        PlanParams(
            deck_m=params.deck_m,
            body_clearance_m=params.body_clearance_m,
            keepout_inflate_m=params.keepout_inflate_m,
            crosscut=params.crosscut,
            grid_cell_m=params.grid_cell_m,
            headland_m=params.headland_m,
            overlap_pct=params.overlap_pct,
            primary_axis=params.primary_axis,
            pattern="boustrophedon",
        ),
    ))
