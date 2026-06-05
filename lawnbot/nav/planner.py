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


def _stripes(
    boundary: Polygon,
    keepouts: list[Polygon],
    deck_m: float,
    inflate_m: float,
    axis: str,
) -> list[list[Point]]:
    """Return a serpentine list of segments along the chosen axis."""
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
        # Trim each interval by inflate_m at each end (boundary clearance).
        ivals = [(a + inflate_m, b - inflate_m) for a, b in ivals if (b - inflate_m) > (a + inflate_m)]
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


def plan_coverage(
    boundary: Polygon,
    keepouts: list[Polygon],
    params: PlanParams,
) -> list[Point]:
    """Build the full waypoint list for the mission."""
    inflate = params.body_clearance_m
    drivable = DrivableMask(boundary, keepouts, cell_m=params.grid_cell_m, inflate_m=inflate)

    waypoints: list[Point] = []

    def append_pass(axis: str) -> None:
        segments = _stripes(boundary, keepouts, params.deck_m, inflate, axis)
        last: Point | None = None
        for seg in segments:
            if last is not None and last != seg[0]:
                connector = _connect(last, seg[0], drivable)
                # Skip the first point (== last) to avoid dup
                waypoints.extend(connector[1:])
            else:
                waypoints.append(seg[0])
            waypoints.append(seg[1])
            last = seg[1]

    append_pass("h")
    if params.crosscut:
        append_pass("v")

    # Drop consecutive duplicates introduced by joins.
    deduped: list[Point] = []
    for p in waypoints:
        if not deduped or dist(deduped[-1], p) > 0.05:
            deduped.append(p)
    return deduped
