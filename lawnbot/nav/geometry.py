"""2-D polygon + line geometry utilities (pure math, no hardware).

All coordinates are local ENU meters. Used by the planner (point-in-polygon
for the drivable grid, interval subtraction for stripe carving) and by the
teach module (Douglas-Peucker simplification, loop-closure detection).
"""
from __future__ import annotations

import math

Point = tuple[float, float]
Polygon = list[Point]


def bbox(poly: Polygon) -> tuple[float, float, float, float]:
    xs = [p[0] for p in poly]
    ys = [p[1] for p in poly]
    return min(xs), min(ys), max(xs), max(ys)


def signed_area(poly: Polygon) -> float:
    a = 0.0
    n = len(poly)
    for i in range(n):
        x1, y1 = poly[i]
        x2, y2 = poly[(i + 1) % n]
        a += x1 * y2 - x2 * y1
    return 0.5 * a


def area(poly: Polygon) -> float:
    return abs(signed_area(poly))


def point_in_polygon(pt: Point, poly: Polygon) -> bool:
    """Ray-cast point-in-polygon. Inclusive of boundary up to FP noise."""
    x, y = pt
    inside = False
    n = len(poly)
    j = n - 1
    for i in range(n):
        xi, yi = poly[i]
        xj, yj = poly[j]
        if ((yi > y) != (yj > y)) and (x < (xj - xi) * (y - yi) / ((yj - yi) or 1e-12) + xi):
            inside = not inside
        j = i
    return inside


def dist2(a: Point, b: Point) -> float:
    dx, dy = a[0] - b[0], a[1] - b[1]
    return dx * dx + dy * dy


def dist(a: Point, b: Point) -> float:
    return math.sqrt(dist2(a, b))


def polyline_length(pts: list[Point]) -> float:
    return sum(dist(pts[i], pts[i + 1]) for i in range(len(pts) - 1))


def point_to_segment_dist(p: Point, a: Point, b: Point) -> float:
    ax, ay = a
    bx, by = b
    dx, dy = bx - ax, by - ay
    L2 = dx * dx + dy * dy
    if L2 < 1e-12:
        return dist(p, a)
    t = ((p[0] - ax) * dx + (p[1] - ay) * dy) / L2
    t = max(0.0, min(1.0, t))
    return dist(p, (ax + t * dx, ay + t * dy))


def signed_distance_to_polygon(p: Point, poly: Polygon) -> float:
    """Positive when ``p`` is inside ``poly``, negative when outside.

    Magnitude is the distance to the nearest polygon edge. Used by the safety
    monitor's geofence so transient noise-driven breaches can be absorbed by a
    configured margin.
    """
    if not poly or len(poly) < 3:
        return float("inf")
    min_d = float("inf")
    n = len(poly)
    for i in range(n):
        d = point_to_segment_dist(p, poly[i], poly[(i + 1) % n])
        if d < min_d:
            min_d = d
    return min_d if point_in_polygon(p, poly) else -min_d


def douglas_peucker(pts: list[Point], eps: float) -> list[Point]:
    """Simplify a polyline to within eps meters."""
    if len(pts) < 3:
        return list(pts)

    def _seg_dist(p: Point, a: Point, b: Point) -> float:
        if a == b:
            return dist(p, a)
        ax, ay = a
        bx, by = b
        px, py = p
        dx, dy = bx - ax, by - ay
        t = ((px - ax) * dx + (py - ay) * dy) / (dx * dx + dy * dy)
        t = max(0.0, min(1.0, t))
        return dist(p, (ax + t * dx, ay + t * dy))

    def _dp(lo: int, hi: int, mask: list[bool]) -> None:
        if hi <= lo + 1:
            return
        a, b = pts[lo], pts[hi]
        max_d, max_i = 0.0, lo
        for i in range(lo + 1, hi):
            d = _seg_dist(pts[i], a, b)
            if d > max_d:
                max_d, max_i = d, i
        if max_d > eps:
            mask[max_i] = True
            _dp(lo, max_i, mask)
            _dp(max_i, hi, mask)

    mask = [False] * len(pts)
    mask[0] = mask[-1] = True
    _dp(0, len(pts) - 1, mask)
    return [pts[i] for i in range(len(pts)) if mask[i]]


def line_clips_polygon_y(y: float, poly: Polygon) -> list[tuple[float, float]]:
    """Return the x-intervals where the horizontal line at height y lies
    inside the polygon. Used for boustrophedon stripe scan-lines.
    """
    xs: list[float] = []
    n = len(poly)
    for i in range(n):
        x1, y1 = poly[i]
        x2, y2 = poly[(i + 1) % n]
        if (y1 <= y < y2) or (y2 <= y < y1):
            xi = x1 + (y - y1) * (x2 - x1) / ((y2 - y1) or 1e-12)
            xs.append(xi)
    xs.sort()
    return [(xs[i], xs[i + 1]) for i in range(0, len(xs) - 1, 2)]


def subtract_intervals(
    intervals: list[tuple[float, float]], cuts: list[tuple[float, float]]
) -> list[tuple[float, float]]:
    """Subtract `cuts` (e.g. keep-out projections) from `intervals`."""
    result = list(intervals)
    for c0, c1 in cuts:
        c0, c1 = min(c0, c1), max(c0, c1)
        out: list[tuple[float, float]] = []
        for a, b in result:
            if c1 <= a or c0 >= b:
                out.append((a, b))
                continue
            if c0 > a:
                out.append((a, c0))
            if c1 < b:
                out.append((c1, b))
        result = out
    return result


def segment_intersects_segment(a1: Point, a2: Point, b1: Point, b2: Point) -> bool:
    def ccw(p: Point, q: Point, r: Point) -> float:
        return (q[0] - p[0]) * (r[1] - p[1]) - (q[1] - p[1]) * (r[0] - p[0])

    d1 = ccw(b1, b2, a1)
    d2 = ccw(b1, b2, a2)
    d3 = ccw(a1, a2, b1)
    d4 = ccw(a1, a2, b2)
    return ((d1 > 0 > d2) or (d1 < 0 < d2)) and ((d3 > 0 > d4) or (d3 < 0 < d4))


def is_simple(poly: Polygon) -> bool:
    """True if no two non-adjacent edges intersect."""
    n = len(poly)
    for i in range(n):
        a1, a2 = poly[i], poly[(i + 1) % n]
        for j in range(i + 1, n):
            if abs(i - j) <= 1 or (i == 0 and j == n - 1):
                continue
            b1, b2 = poly[j], poly[(j + 1) % n]
            if segment_intersects_segment(a1, a2, b1, b2):
                return False
    return True


def los_clear(a: Point, b: Point, drivable: "DrivableMask", step_m: float = 0.05) -> bool:
    """Sample the segment a→b; clear if every sample lies on the drivable mask."""
    d = dist(a, b)
    if d == 0:
        return drivable.at(*a)
    n = max(2, int(d / step_m))
    for i in range(n + 1):
        t = i / n
        x = a[0] + t * (b[0] - a[0])
        y = a[1] + t * (b[1] - a[1])
        if not drivable.at(x, y):
            return False
    return True


class DrivableMask:
    """Grid mask of lawn-minus-keepouts-minus-body-clearance.

    Built once at mission start from the boundary polygon and keep-out
    polygons. The planner uses it for LOS and A* connectors.
    """

    def __init__(self, boundary: Polygon, keepouts: list[Polygon], cell_m: float, inflate_m: float):
        self.cell = cell_m
        x0, y0, x1, y1 = bbox(boundary)
        self.x0, self.y0 = x0, y0
        self.nx = max(1, int(math.ceil((x1 - x0) / cell_m)) + 1)
        self.ny = max(1, int(math.ceil((y1 - y0) / cell_m)) + 1)
        self._grid = [[False] * self.nx for _ in range(self.ny)]
        # Coarse mark: inside boundary, outside every keep-out.
        for j in range(self.ny):
            for i in range(self.nx):
                x = x0 + i * cell_m
                y = y0 + j * cell_m
                if not point_in_polygon((x, y), boundary):
                    continue
                if any(point_in_polygon((x, y), ko) for ko in keepouts):
                    continue
                self._grid[j][i] = True
        # Inflate: shrink free space by inflate_m (body clearance).
        if inflate_m > 0:
            r_cells = int(math.ceil(inflate_m / cell_m))
            blocked: list[tuple[int, int]] = []
            for j in range(self.ny):
                for i in range(self.nx):
                    if not self._grid[j][i]:
                        for dj in range(-r_cells, r_cells + 1):
                            for di in range(-r_cells, r_cells + 1):
                                if di * di + dj * dj <= r_cells * r_cells:
                                    blocked.append((j + dj, i + di))
            for j, i in blocked:
                if 0 <= j < self.ny and 0 <= i < self.nx:
                    self._grid[j][i] = False

    def at(self, x: float, y: float) -> bool:
        i = int(round((x - self.x0) / self.cell))
        j = int(round((y - self.y0) / self.cell))
        if 0 <= i < self.nx and 0 <= j < self.ny:
            return self._grid[j][i]
        return False

    def ij_to_xy(self, i: int, j: int) -> Point:
        return self.x0 + i * self.cell, self.y0 + j * self.cell
