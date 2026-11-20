"""Unit tests for the simulator-independent logic (run with colcon test)."""
import math
import os

import numpy as np
import pytest

from tins8_mission import geo
from tins8_mission.guarded_exchange import (A_UPPER_INDEX, B_UPPER_INDEX, Sequencer, load_waypoints, route,
                                            station_of)
from tins8_mission.ibvs import IbvsParams, estimate_range, ibvs_step
from tins8_mission.tracking import IouTracker

HERE = os.path.dirname(__file__)
WAYPOINTS = os.path.join(HERE, '..', '..', 'tins_description', 'config', 'guarded_exchange_waypoints.yaml')
LEVEL = [1.0, 0.0, 0.0, 0.0]


def yaw_q(heading):
    return [math.cos(heading / 2), 0.0, 0.0, math.sin(heading / 2)]


def pitch_q(pitch):
    return [math.cos(pitch / 2), 0.0, math.sin(pitch / 2), 0.0]


def test_ltp_round_trip():
    ltp = geo.LocalTangentPlane(47.397742, 8.545594, 488.0)
    lat, lon, alt = ltp.to_geodetic(25.0, -40.0, 7.0)
    assert np.allclose(ltp.to_enu(lat, lon, alt), [25.0, -40.0, 7.0], atol=1e-6)


def test_enu_ned_and_yaw():
    assert np.allclose(geo.enu_to_ned([1, 2, 3]), [2, 1, -3])
    assert np.allclose(geo.ned_to_enu(geo.enu_to_ned([4, -5, 6])), [4, -5, 6])
    assert math.isclose(geo.heading_ned_to_yaw_enu(0.0), math.pi / 2)        # north
    assert math.isclose(geo.yaw_enu_to_heading_ned(0.0), math.pi / 2)        # east


def test_centre_pixel_on_level_vehicle_looks_down_by_tilt():
    tilt = math.radians(20)
    ray = geo.pixel_to_level_ray(320, 240, 270, 270, 320, 240, tilt, LEVEL)
    b, e = geo.bearing_elevation(ray)
    assert abs(b) < 1e-9 and math.isclose(e, tilt, abs_tol=1e-9)


def test_pixel_right_of_centre_is_positive_bearing_for_any_heading():
    for heading in (0.0, 1.0, -2.5):
        ray = geo.pixel_to_level_ray(500, 240, 270, 270, 320, 240, 0.0, yaw_q(heading))
        b, _ = geo.bearing_elevation(ray)
        assert math.isclose(b, math.atan2(180, 270), abs_tol=1e-9)


def test_pitch_compensation():
    """Nose-down pitch moves a fixed ground point up in the image; the level ray must not change."""
    tilt = math.radians(20)
    pitch = math.radians(-15)          # PX4 FRD: negative pitch = nose down
    # target 30 deg below horizon, straight ahead
    level_target = np.array([math.cos(math.radians(30)), 0.0, math.sin(math.radians(30))])
    r = geo.quat_to_matrix(pitch_q(pitch))
    frd = r.T @ level_target
    flu = np.array([frd[0], -frd[1], -frd[2]])
    c, s = math.cos(tilt), math.sin(tilt)
    cam = np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]]).T @ flu
    u = 320 + 270 * (-cam[1] / cam[0])
    v = 240 + 270 * (-cam[2] / cam[0])
    ray = geo.pixel_to_level_ray(u, v, 270, 270, 320, 240, tilt, pitch_q(pitch))
    _, e = geo.bearing_elevation(ray)
    assert math.isclose(e, math.radians(30), abs_tol=1e-6)


def test_range_estimate_and_standoff():
    fy = 270.0
    assert math.isclose(estimate_range(fy, 0.65 * 480, 1.75), 1.514, abs_tol=0.01)


def test_ibvs_closes_far_and_holds_at_standoff():
    p = IbvsParams()
    far = ibvs_step(p, 0.0, p.camera_tilt_rad, 15.0, 480, 270.0, 1.6, False)     # 31 m, already low
    assert math.hypot(far.vx_fwd, far.vz_down) > 0.9 * p.max_vel and not far.terminal   # full speed along the LOS
    high = ibvs_step(p, 0.0, p.camera_tilt_rad, 30.0, 480, 270.0, 8.0, False)    # centred, 16 m
    assert high.vz_down > 0 and abs(high.vz_down / high.vx_fwd - math.tan(p.camera_tilt_rad)) < 0.02  # LOS
    late = ibvs_step(p, 0.0, p.camera_tilt_rad, 120.0, 480, 270.0, 1.6, False, v_close_meas=4.0, meas_age_s=0.4)
    assert late.vx_fwd < 0.5          # 3.9 m away in a 0.4 s old image at 4 m/s: brake now
    clip = ibvs_step(p, 0.0, p.camera_tilt_rad, 0.45 * 480, 480, 270.0, 1.5, False, clipped=True)
    assert clip.terminal and clip.vx_fwd < 0      # clipped tall box: too close, back away
    close = ibvs_step(p, 0.0, p.camera_tilt_rad, 120.0, 480, 270.0, 1.6, False)  # 3.9 m, low
    assert close.vx_fwd <= math.sqrt(2 * p.brake_accel * 2.5) + 1e-6             # braking limit
    floor = ibvs_step(p, 0.0, math.radians(40), 60.0, 480, 270.0, p.min_alt_m + 0.2, False)
    assert floor.vz_down <= math.sqrt(2 * p.brake_accel * 0.2) + 1e-6             # floor anticipated
    near = ibvs_step(p, 0.0, p.camera_tilt_rad, 0.7 * 480, 480, 270.0, 1.5, False)
    assert near.terminal and near.vx_fwd == 0.0


def test_ibvs_yaws_towards_target_and_respects_floor():
    p = IbvsParams()
    c = ibvs_step(p, 0.3, p.camera_tilt_rad + 0.4, 60.0, 480, 270.0, p.min_alt_m, False)
    assert c.yawspeed > 0 and c.vy_right > 0
    assert c.vz_down == 0.0            # wants to descend but sits on the altitude floor
    low = ibvs_step(p, 0.0, p.camera_tilt_rad, 60.0, 480, 270.0, p.min_alt_m - 0.3, False)
    assert low.vz_down < 0             # climbs back above the floor


def test_tracker_keeps_ids_and_never_reuses_them():
    trk = IouTracker(min_hits=1)
    a, b = [100, 100, 140, 220], [400, 120, 440, 240]
    t1 = trk.update([a, b], [0.9, 0.9], 0.0)
    ids = {tr.tid for tr in t1}
    t2 = trk.update([[104, 100, 144, 220], [404, 120, 444, 240]], [0.9, 0.9], 0.1)
    assert {tr.tid for tr in t2} == ids
    t3 = trk.update([[408, 120, 448, 240]], [0.9], 1.5)        # first person gone > max_age
    t4 = trk.update([[100, 100, 140, 220], [412, 120, 452, 240]], [0.9, 0.9], 1.6)
    new_ids = {tr.tid for tr in t4}
    assert len(new_ids & ids) == 1 and max(new_ids) > max(ids)  # returning person gets a NEW id


@pytest.mark.skipif(not os.path.exists(WAYPOINTS), reason='tins_description not next to this package')
def test_guarded_route_is_gated_on_measurement():
    w = load_waypoints(WAYPOINTS)
    assert len(w) == 26
    assert 'uav01_dock' in w[A_UPPER_INDEX].accessible and 'uav02_dock' in w[B_UPPER_INDEX].accessible
    seq = Sequencer(w)
    target = seq.start(route(A_UPPER_INDEX, B_UPPER_INDEX))
    assert target == w[1].q
    # no measurement / stale measurement / not yet reached: target must not advance
    assert seq.update(w[0].q, True)[1] is None
    assert seq.update(w[1].q, False)[1] is None
    _, idx, done = seq.update(w[1].q, True)
    assert idx == 1 and not done
    for i in range(2, 13):
        _, idx, done = seq.update(w[i].q, True)
        assert idx == i
    assert done
    assert station_of(w[12].q, w, 0.0015) == ('B_UPPER', 12)


def test_route_rejects_unknown_moves():
    with pytest.raises(ValueError):
        route(3, 12)


def test_rotation_warp_is_identity_without_rotation_and_follows_yaw():
    q0 = yaw_q(0.3)
    w = geo.rotation_warp(270, 270, 320, 240, math.radians(20), q0, q0)
    u, v = w(100.0, 300.0)
    assert abs(u - 100.0) < 1e-6 and abs(v - 300.0) < 1e-6
    # yawing right moves static scene points left in the image
    w = geo.rotation_warp(270, 270, 320, 240, math.radians(20), q0, yaw_q(0.3 + 0.2))
    u2, _ = w(320.0, 240.0)
    assert u2 < 320.0 - 40.0
    # pixel -> ray -> pixel round trip
    ray = geo.pixel_to_ned_ray(410, 170, 270, 270, 320, 240, math.radians(20), pitch_q(math.radians(-8)))
    p = geo.ned_ray_to_pixel(ray, 270, 270, 320, 240, math.radians(20), pitch_q(math.radians(-8)))
    assert abs(p[0] - 410) < 1e-6 and abs(p[1] - 170) < 1e-6


def test_tracker_keeps_id_through_fast_yaw_with_warp():
    trk = IouTracker(min_hits=1)
    box = [300, 200, 330, 280]
    trk.update([box], [0.9], 0.0)
    shift = -150.0                     # aircraft yawed; the person jumped 150 px left
    moved = [box[0] + shift, box[1], box[2] + shift, box[3]]
    plain = IouTracker(min_hits=1)
    plain.update([box], [0.9], 0.0)
    assert plain.update([moved], [0.9], 0.25)[0].tid != 1          # without compensation: new ID
    out = trk.update([moved], [0.9], 0.25, warp=lambda u, v: (u + shift, v))
    assert out[0].tid == 1                                         # with compensation: same ID
