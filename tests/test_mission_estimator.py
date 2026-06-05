import math

from lawnbot.config import ControlCfg, EstimatorCfg, PIDGains, PurePursuit
from lawnbot.estimator import Estimator
from lawnbot.gnss.lc29h import GnssFix
from lawnbot.nav.geo import Origin
from lawnbot.nav.mission import Mission, State


def _ctrl():
    return ControlCfg(
        ctrl_hz=20, v_nominal=0.4, reach_m=0.2, max_steer_deg=30,
        pure_pursuit=PurePursuit(enabled=True, lookahead_m=0.6),
        pid=PIDGains(kp=1, ki=0, kd=0, imax=1),
    )


def test_estimator_dead_reckon_straight():
    est = Estimator(EstimatorCfg(gps_blend_alpha=0.5, imu_yaw_offset_deg=0.0),
                    Origin(lat=45.0, lon=-73.0))
    for _ in range(10):
        est.dead_reckon(yaw_rad=0.0, ds_m=0.1)
    p = est.snapshot()
    assert math.isclose(p.x, 1.0, abs_tol=1e-9)
    assert math.isclose(p.y, 0.0, abs_tol=1e-9)


def test_estimator_gps_blends():
    est = Estimator(EstimatorCfg(gps_blend_alpha=0.5, imu_yaw_offset_deg=0.0),
                    Origin(lat=45.0, lon=-73.0))
    est.seed(10.0, 0.0)
    fake_fix = GnssFix(lat=45.0, lon=-73.0, quality=4, sats=12, hdop=1.0, alt_m=0, timestamp_mono=0)
    est.ingest_gps(fake_fix)
    p = est.snapshot()
    # ENU origin at the same lat/lon → GPS = (0,0); 0.5 blend from 10 → 5.
    assert math.isclose(p.x, 5.0, abs_tol=1e-3)


def test_mission_advances_when_within_reach():
    m = Mission(_ctrl())
    m.load_path([(1.0, 0.0), (2.0, 0.0), (3.0, 0.0)])
    m.start()
    from lawnbot.estimator import Pose
    target, done = m.update(Pose(x=0.95, y=0.0))
    # Within reach (0.2) of (1, 0) — advances to (2, 0).
    assert not done
    assert target == (2.0, 0.0)


def test_mission_done():
    m = Mission(_ctrl())
    m.load_path([(0.1, 0.0)])
    m.start()
    from lawnbot.estimator import Pose
    target, done = m.update(Pose(x=0.0, y=0.0))
    assert done
    assert m.snapshot().state == State.DONE
