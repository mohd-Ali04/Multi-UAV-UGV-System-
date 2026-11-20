"""Frame conversions used across the project (pure Python, unit tested).

Frames
  world ENU : Gazebo world frame, x east, y north, z up (metres).
  PX4 NED   : each PX4 instance's local frame, x north, y east, z down, origin at the
              EKF reference (ref_lat, ref_lon, ref_alt from vehicle_local_position).
  body FRD  : PX4 body frame, x forward, y right, z down.
  camera    : Gazebo camera link, x forward (optical axis), y left, z up.
  optical   : image convention, x right, y down, z forward.
"""
import math

import numpy as np

WGS84_A = 6378137.0
WGS84_E2 = 6.69437999014e-3


class LocalTangentPlane:
    """Small-area geodetic <-> ENU conversion about the Gazebo world origin."""

    def __init__(self, lat0_deg, lon0_deg, alt0_m):
        self.lat0 = math.radians(lat0_deg)
        self.lon0 = math.radians(lon0_deg)
        self.alt0 = alt0_m
        s = math.sin(self.lat0)
        self.r_n = WGS84_A / math.sqrt(1 - WGS84_E2 * s * s)                  # prime vertical
        self.r_m = self.r_n * (1 - WGS84_E2) / (1 - WGS84_E2 * s * s)          # meridian

    def to_enu(self, lat_deg, lon_deg, alt_m):
        e = (math.radians(lon_deg) - self.lon0) * self.r_n * math.cos(self.lat0)
        n = (math.radians(lat_deg) - self.lat0) * self.r_m
        return np.array([e, n, alt_m - self.alt0])

    def to_geodetic(self, e, n, u):
        lat = self.lat0 + n / self.r_m
        lon = self.lon0 + e / (self.r_n * math.cos(self.lat0))
        return math.degrees(lat), math.degrees(lon), u + self.alt0


def ned_to_enu(v):
    return np.array([v[1], v[0], -v[2]])


def enu_to_ned(v):
    return np.array([v[1], v[0], -v[2]])


def heading_ned_to_yaw_enu(heading):
    return wrap(math.pi / 2 - heading)


def yaw_enu_to_heading_ned(yaw):
    return wrap(math.pi / 2 - yaw)


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def quat_to_matrix(q):
    """PX4 quaternion [w, x, y, z] (FRD body -> NED) to rotation matrix."""
    w, x, y, z = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def euler_from_matrix(r):
    """ZYX (yaw, pitch, roll) of a body->NED rotation."""
    pitch = math.asin(max(-1.0, min(1.0, -r[2, 0])))
    roll = math.atan2(r[2, 1], r[2, 2])
    yaw = math.atan2(r[1, 0], r[0, 0])
    return yaw, pitch, roll


def _cam_to_body_frd(tilt_down_rad):
    """Rotation camera link FLU -> body FRD for a camera pitched nose-down by tilt."""
    c, s = math.cos(tilt_down_rad), math.sin(tilt_down_rad)
    ry = np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])      # camera FLU -> body FLU
    flu_to_frd = np.diag([1.0, -1.0, -1.0])
    return flu_to_frd @ ry


def pixel_to_ned_ray(u, v, fx, fy, cx, cy, tilt_down_rad, q_frd_to_ned):
    """Unit ray to pixel (u, v) in the local NED frame for a given vehicle attitude."""
    # optical (x right, y down, z fwd) -> camera link FLU (x fwd, y left, z up)
    xo, yo = (u - cx) / fx, (v - cy) / fy
    cam = np.array([1.0, -xo, -yo])
    ned = quat_to_matrix(q_frd_to_ned) @ (_cam_to_body_frd(tilt_down_rad) @ cam)
    return ned / np.linalg.norm(ned)


def ned_ray_to_pixel(ray_ned, fx, fy, cx, cy, tilt_down_rad, q_frd_to_ned):
    """Inverse of pixel_to_ned_ray; None if the ray points behind the camera."""
    cam = _cam_to_body_frd(tilt_down_rad).T @ (quat_to_matrix(q_frd_to_ned).T @ np.asarray(ray_ned))
    if cam[0] <= 1e-6:
        return None
    return cx - fx * cam[1] / cam[0], cy - fy * cam[2] / cam[0]


def rotation_warp(fx, fy, cx, cy, tilt_down_rad, q_from, q_to):
    """Pixel map for a pure camera rotation between two attitudes (ego-motion compensation).

    A static scene point seen at (u, v) with attitude q_from appears at warp(u, v) with
    attitude q_to.  Translation is ignored, which is accurate for the large, fast image shifts
    produced by yaw and pitch changes between two detector frames.
    """
    def warp(u, v):
        p = ned_ray_to_pixel(pixel_to_ned_ray(u, v, fx, fy, cx, cy, tilt_down_rad, q_from),
                             fx, fy, cx, cy, tilt_down_rad, q_to)
        return (u, v) if p is None else p
    return warp


def pixel_to_level_ray(u, v, fx, fy, cx, cy, tilt_down_rad, q_frd_to_ned):
    """Unit ray to pixel (u, v) expressed in a yaw-aligned, gravity-level frame.

    Returned frame: x forward (horizontal, along vehicle heading), y right, z down.
    Removing roll and pitch here is the pitch compensation: aggressive forward
    acceleration tilts the camera, but the bearing/elevation computed from this
    ray stay referenced to the horizon.
    """
    ned = pixel_to_ned_ray(u, v, fx, fy, cx, cy, tilt_down_rad, q_frd_to_ned)
    yaw, _, _ = euler_from_matrix(quat_to_matrix(q_frd_to_ned))
    cy_, sy_ = math.cos(yaw), math.sin(yaw)
    level = np.array([cy_ * ned[0] + sy_ * ned[1], -sy_ * ned[0] + cy_ * ned[1], ned[2]])
    return level / np.linalg.norm(level)


def bearing_elevation(level_ray):
    """Bearing (+ right) and elevation below the horizon (+ down) in radians."""
    x, y, z = level_ray
    return math.atan2(y, x), math.atan2(z, math.hypot(x, y))
