"""Image-based visual servoing law for rapid pursuit of one YOLOX person track.

Pure function of the current measurement so it can be unit tested.  The ROS wrapper
(pursuit_node.py) handles timing, failsafes, PX4 topics and logging.

Measurements (from the selected detection):
  bearing  : horizontal angle of the bbox centre, + right, from a pitch/roll-compensated ray
  elev     : angle below the horizon of the bbox centre
  h_frac   : bbox height / image height
Range is estimated from the pinhole model with an average person height:
  range = fy * H_person / h_px          (fy from camera_info)
"""
import math
from dataclasses import dataclass


@dataclass
class IbvsParams:
    person_height_m: float = 1.75
    max_vel: float = 6.0          # m/s forward closing limit
    max_reverse: float = 1.0
    max_lateral: float = 2.0
    max_vz: float = 2.5
    max_yawspeed: float = 1.2     # rad/s (camera turns <= ~0.3 rad between ~4 Hz detections)
    k_yaw: float = 2.0
    k_lat: float = 0.6            # lateral m/s per (rad * m) of bearing*range
    k_fwd: float = 0.9            # forward m/s per metre of range error
    k_z: float = 1.0              # vertical m/s per rad of elevation error, scaled by range
    standoff_frac: float = 0.65   # terminal when bbox height fills this fraction of the frame
    resume_frac: float = 0.55     # leave terminal hover below this fraction (hysteresis)
    backoff_frac: float = 0.8     # in terminal hover, back away above this fraction
    clip_frac: float = 0.35       # a box clipped by the frame edge and this tall = too close
    min_alt_m: float = 1.2        # hard altitude floor (above ground)
    camera_tilt_rad: float = math.radians(20.0)
    bearing_slowdown_rad: float = 0.6   # forward speed tapers to zero at this bearing error
    response_s: float = 0.5       # velocity response lag of the vehicle (PX4 + slew), s
    brake_accel: float = 1.5      # m/s^2 for the braking-distance limits (gentle: braking
    #                               pitches the nose and the camera up)
    elev_deadband_rad: float = math.radians(5.0)
    elev_slowdown_rad: float = math.radians(20.0)  # forward speed tapers to zero when the target
    #                                                is this far below the deadband (glide slope:
    #                                                do not overrun the target before descending)


@dataclass
class IbvsCommand:
    vx_fwd: float
    vy_right: float
    vz_down: float
    yawspeed: float
    range_est: float
    terminal: bool


def estimate_range(fy, h_px, person_height_m):
    return fy * person_height_m / max(h_px, 1.0)


def ibvs_step(p: IbvsParams, bearing, elev, h_px, img_h, fy, altitude_m, terminal_prev,
              v_close_meas=0.0, meas_age_s=0.0, clipped=False):
    """One control step.  Returns level-frame velocities (forward, right, down) and yaw rate.

    Closing is commanded ALONG THE LINE OF SIGHT to the box centre: forward = v cos(elev),
    down = v sin(elev).  Flying the line of sight keeps the elevation angle constant, so the
    aircraft descends from its observation height at the rate the geometry needs and arrives
    low enough to see the whole person at the standoff range (camera tilted 20 deg down).
    A feedback term on the elevation error keeps the box on the optical axis.

    v_close_meas / meas_age_s: measured closing speed along the line of sight and the age of
    the image the box came from.  The range is propagated to "now" and the braking limit also
    covers the vehicle's velocity response time, so a fast approach stops at the standoff
    instead of overrunning the person between two detector frames.

    clipped: the box touches the top or bottom image edge.  Its height then under-reads the
    person and the range estimate is too long exactly when the aircraft is closest, so a
    clipped box that already fills clip_frac of the frame is treated as inside the standoff.
    """
    rng = estimate_range(fy, h_px, p.person_height_m)
    h_frac = h_px / img_h
    too_close = clipped and h_frac >= p.clip_frac
    terminal = too_close or h_frac >= p.standoff_frac or (terminal_prev and h_frac >= p.resume_frac)
    standoff_range = estimate_range(fy, p.standoff_frac * img_h, p.person_height_m)

    yawspeed = max(-p.max_yawspeed, min(p.max_yawspeed, p.k_yaw * bearing))
    vy = max(-p.max_lateral, min(p.max_lateral, p.k_lat * bearing * rng))

    el_err = elev - p.camera_tilt_rad            # + target lower than the optical axis
    if too_close:
        v_los = -0.5 * p.max_reverse
    elif terminal:
        # Hold the standoff; back away if the person walks into the aircraft.
        v_los = -min(p.max_reverse, p.k_fwd * max(0.0, standoff_range - rng)) if h_frac > p.backoff_frac else 0.0
    elif rng < standoff_range:
        v_los = -min(p.max_reverse, p.k_fwd * (standoff_range - rng))
    else:
        v_meas = max(0.0, v_close_meas)
        d_go = max(0.0, rng - v_meas * meas_age_s - standoff_range)    # range propagated to now
        d_brake = max(0.0, d_go - v_meas * p.response_s)               # minus the response lag
        v_los = p.k_fwd * d_go
        v_los = min(v_los, math.sqrt(2.0 * p.brake_accel * d_brake))   # braking distance
        v_los *= max(0.0, 1.0 - abs(bearing) / p.bearing_slowdown_rad)  # face the target first
        below = el_err - p.elev_deadband_rad                            # target sliding off the
        v_los *= min(1.0, max(0.0, 1.0 - below / p.elev_slowdown_rad))  # bottom: slow down
        sin_el = max(0.05, math.sin(max(0.0, elev)))
        v_los = min(v_los, p.max_vel, 0.8 * p.max_vz / sin_el)           # vertical capacity
    vx = v_los * math.cos(elev)
    vz = v_los * math.sin(elev) + p.k_z * el_err * max(1.0, min(rng, 10.0))
    vz = max(-p.max_vz, min(p.max_vz, vz))
    # Altitude floor, anticipated: never descend faster than can be stopped before the floor.
    margin = altitude_m - p.min_alt_m
    vz = min(vz, math.sqrt(2.0 * p.brake_accel * max(0.0, margin)))
    if altitude_m < p.min_alt_m - 0.1:
        vz = -0.5                                # climb back above the floor
    vx = max(-p.max_reverse, min(p.max_vel, vx))
    return IbvsCommand(vx, vy, vz, yawspeed, rng, terminal)


def slew(prev, target, max_step):
    return prev + max(-max_step, min(max_step, target - prev))
