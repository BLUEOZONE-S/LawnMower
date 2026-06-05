import math

from lawnbot.nav.geometry import (
    DrivableMask,
    area,
    douglas_peucker,
    is_simple,
    line_clips_polygon_y,
    point_in_polygon,
    polyline_length,
    subtract_intervals,
)


def square(side: float = 10.0):
    return [(0.0, 0.0), (side, 0.0), (side, side), (0.0, side)]


def test_point_in_polygon_square():
    sq = square()
    assert point_in_polygon((5, 5), sq)
    assert not point_in_polygon((-1, 5), sq)
    assert not point_in_polygon((11, 5), sq)


def test_area_of_square():
    assert math.isclose(area(square(7.0)), 49.0)


def test_dp_simplifies_redundant_points():
    pts = [(0, 0), (1, 0.001), (2, -0.002), (3, 0.001), (4, 0)]
    simplified = douglas_peucker(pts, eps=0.01)
    assert len(simplified) == 2
    assert simplified[0] == (0, 0)
    assert simplified[-1] == (4, 0)


def test_dp_preserves_corners():
    pts = [(0, 0), (5, 0), (5, 5), (0, 5)]
    assert douglas_peucker(pts, 0.1) == pts


def test_polyline_length():
    assert math.isclose(polyline_length([(0, 0), (3, 0), (3, 4)]), 7.0)


def test_line_clips_simple():
    ivs = line_clips_polygon_y(5.0, square())
    assert len(ivs) == 1
    assert math.isclose(ivs[0][0], 0)
    assert math.isclose(ivs[0][1], 10)


def test_interval_subtraction():
    ivs = subtract_intervals([(0, 10)], [(3, 4), (7, 8)])
    assert ivs == [(0, 3), (4, 7), (8, 10)]


def test_is_simple():
    assert is_simple([(0, 0), (1, 0), (1, 1), (0, 1)])
    # Bowtie:
    assert not is_simple([(0, 0), (1, 1), (1, 0), (0, 1)])


def test_drivable_mask_blocks_outside_boundary():
    mask = DrivableMask(square(10.0), keepouts=[], cell_m=0.5, inflate_m=0.0)
    assert mask.at(5, 5) is True
    assert mask.at(-1, 5) is False
    assert mask.at(11, 5) is False


def test_drivable_mask_carves_keepout():
    keepout = [(4.0, 4.0), (6.0, 4.0), (6.0, 6.0), (4.0, 6.0)]
    mask = DrivableMask(square(10.0), keepouts=[keepout], cell_m=0.5, inflate_m=0.0)
    assert mask.at(5, 5) is False
    assert mask.at(2, 2) is True
