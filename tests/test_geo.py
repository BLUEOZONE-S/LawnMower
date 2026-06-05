import math

from lawnbot.nav.geo import Origin, centroid_ll, to_enu, to_ll


def test_origin_roundtrip():
    origin = Origin(lat=45.0, lon=-73.0)
    east, north = to_enu(45.001, -73.001, origin)
    # 0.001° lat ≈ 110.54 m, 0.001° lon at 45° ≈ 78.7 m
    assert math.isclose(north, 110.54, rel_tol=1e-3)
    assert math.isclose(east, -78.71, rel_tol=1e-2)
    lat, lon = to_ll(east, north, origin)
    assert math.isclose(lat, 45.001, abs_tol=1e-9)
    assert math.isclose(lon, -73.001, abs_tol=1e-9)


def test_centroid_ll():
    o = centroid_ll([(45.0, -73.0), (45.001, -73.001), (45.0005, -73.0005)])
    assert math.isclose(o.lat, 45.0005, abs_tol=1e-6)
    assert math.isclose(o.lon, -73.0005, abs_tol=1e-6)
