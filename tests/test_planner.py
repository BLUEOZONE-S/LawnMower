from lawnbot.nav.planner import PlanParams, plan_coverage


def test_plan_covers_open_square():
    boundary = [(0.0, 0.0), (10.0, 0.0), (10.0, 10.0), (0.0, 10.0)]
    params = PlanParams(deck_m=0.5, body_clearance_m=0.2, keepout_inflate_m=0.2, crosscut=False)
    path = plan_coverage(boundary, keepouts=[], params=params)
    assert len(path) > 10  # boustrophedon stripes generate many waypoints
    # All path points should be inside the boundary (modulo body clearance trim).
    for x, y in path:
        assert -0.5 <= x <= 10.5
        assert -0.5 <= y <= 10.5


def test_plan_with_keepout_avoids_it():
    boundary = [(0.0, 0.0), (10.0, 0.0), (10.0, 10.0), (0.0, 10.0)]
    keepout = [(4.0, 4.0), (6.0, 4.0), (6.0, 6.0), (4.0, 6.0)]
    params = PlanParams(deck_m=0.5, body_clearance_m=0.2, keepout_inflate_m=0.2, crosscut=False)
    path = plan_coverage(boundary, keepouts=[keepout], params=params)
    # No waypoint should land deep inside the keepout (inside by > body_clearance).
    for x, y in path:
        if 4.5 <= x <= 5.5 and 4.5 <= y <= 5.5:
            raise AssertionError(f"path enters keepout interior: ({x}, {y})")
