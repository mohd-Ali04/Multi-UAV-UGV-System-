"""High-speed IBVS pursuit and rapid closing on a selected YOLOX person track.

Engage from a separate terminal:
    ros2 service call /uav01/pursuit/command tins8_interfaces/srv/PursuitCommand "{engage: true, track_id: 3}"
or  tins8ctl pursuit uav01 start 3

On engage the node takes setpoint authority from uav_agent (/uav01/control_owner = 'pursuit')
and streams OffboardControlMode(velocity) + TrajectorySetpoint at 30 Hz:
  * yaw rate and lateral body velocity centre the box horizontally,
  * forward velocity closes the range estimated from box height (fy, 1.75 m person),
  * vertical velocity keeps the box on the optical axis, with a hard altitude floor,
  * all angles come from a pitch/roll-compensated ray (camera optical -> body FRD -> NED),
  * terminal hover when the box fills standoff_frac of the frame height,
  * selected ID absent > lost_after_s -> zero velocity hover, then a slow search yaw,
    then hand control back to the agent (position hold).  It never switches IDs.
Every engagement writes a CSV log and a JSON summary under $TINS8_LOG_DIR/pursuit/.
"""
import csv
import json
import math
import os
import time

import numpy as np
import rclpy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleAttitude, VehicleLocalPosition
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import String
from vision_msgs.msg import Detection2DArray

from tins8_interfaces.msg import UavStatus
from tins8_interfaces.srv import PursuitCommand

from .common import AttitudeHistory, CommandLog, LATCHED_QOS, PX4_OUT_QOS, px4_topic, new_request_id, scenario
from .geo import bearing_elevation, euler_from_matrix, pixel_to_level_ray, pixel_to_ned_ray, quat_to_matrix, wrap
from .ibvs import IbvsParams, ibvs_step, slew


class PursuitNode(Node):
    def __init__(self):
        super().__init__('pursuit_node')
        cfg = scenario()
        self.declare_parameter('uav_id', 'uav01')
        self.declare_parameter('px4_namespace', '')
        self.declare_parameter('rate_hz', 30.0)
        self.declare_parameter('max_vel', 6.0)
        self.declare_parameter('max_accel', 4.0)
        self.declare_parameter('standoff_frac', 0.65)
        self.declare_parameter('min_alt_m', 1.2)
        self.declare_parameter('lost_after_s', 0.5)
        self.declare_parameter('search_yaw_rate', 0.3)
        self.declare_parameter('search_timeout_s', 15.0)
        self.declare_parameter('ground_z_enu', 0.0)
        self.uav = self.get_parameter('uav_id').value
        ns = self.get_parameter('px4_namespace').value or f'/{self.uav}'
        self.p = IbvsParams(max_vel=self.get_parameter('max_vel').value,
                            standoff_frac=self.get_parameter('standoff_frac').value,
                            min_alt_m=self.get_parameter('min_alt_m').value,
                            camera_tilt_rad=math.radians(cfg['camera']['tilt_down_deg']))
        self.log = CommandLog(self, f'{self.uav}/pursuit')
        self.cam = None
        self.att = None
        self.lpos = None
        self.status = None
        self.det = None
        self.det_seen_t = None
        self.det_stamp = None
        self.att_hist = AttitudeHistory()
        self.active = False
        self.state = 'IDLE'
        self.track_id = -1
        self.req = None
        self.cmd_v = np.zeros(3)      # body-level forward, right, down
        self.terminal = False
        self.prev_range = None
        self.prev_det_stamp = None
        self.closing = None
        self.metrics = None
        self.pub_mode = self.create_publisher(OffboardControlMode, px4_topic(ns, 'in', 'offboard_control_mode', OffboardControlMode), 10)
        self.pub_sp = self.create_publisher(TrajectorySetpoint, px4_topic(ns, 'in', 'trajectory_setpoint', TrajectorySetpoint), 10)
        self.pub_owner = self.create_publisher(String, f'/{self.uav}/control_owner', LATCHED_QOS)
        self.pub_state = self.create_publisher(String, f'/{self.uav}/pursuit/state', 10)
        self.create_subscription(CameraInfo, f'/{self.uav}/camera/camera_info', self.on_cam, 10)
        self.create_subscription(VehicleAttitude, px4_topic(ns, 'out', 'vehicle_attitude', VehicleAttitude), self.on_att, PX4_OUT_QOS)
        self.create_subscription(VehicleLocalPosition, px4_topic(ns, 'out', 'vehicle_local_position', VehicleLocalPosition), lambda m: setattr(self, 'lpos', m), PX4_OUT_QOS)
        self.create_subscription(UavStatus, f'/{self.uav}/status', lambda m: setattr(self, 'status', m), 10)
        self.create_subscription(Detection2DArray, f'/{self.uav}/detections', self.on_dets, 10)
        self.create_service(PursuitCommand, f'/{self.uav}/pursuit/command', self.on_command)
        self.create_timer(1.0 / self.get_parameter('rate_hz').value, self.tick)

    def now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_cam(self, m):
        self.cam = m

    def on_att(self, m):
        self.att = m
        self.att_hist.add(self.now_s(), m.q)

    def on_dets(self, msg):
        self.last_ids = {int(d.id): d for d in msg.detections}
        self.last_ids_t = self.now_s()
        for d in msg.detections:
            if int(d.id) == self.track_id:
                self.det = d
                self.det_seen_t = self.now_s()
                self.det_stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

    # ------------------------------------------------------------- commands
    def on_command(self, req, res):
        res.request_id = new_request_id('pur')
        cmd = 'ENGAGE' if req.engage else 'DISENGAGE'
        self.log.emit(res.request_id, cmd, 'REQUESTED', argument=req.track_id)
        if req.engage:
            if self.status is None or self.status.phase not in ('HOLD', 'TRANSIT'):
                res.accepted, res.message = False, f'aircraft must be airborne in HOLD (now {self.status.phase if self.status else "no status"})'
            elif self.cam is None or self.att is None or self.lpos is None:
                res.accepted, res.message = False, 'missing camera_info or PX4 attitude/position'
            elif req.track_id not in getattr(self, 'last_ids', {}) or self.now_s() - self.last_ids_t > 1.0:
                res.accepted, res.message = False, f'track {req.track_id} is not currently detected'
            elif self.active:
                res.accepted, res.message = False, f'already pursuing track {self.track_id}; disengage first'
            else:
                self.det, self.det_seen_t = self.last_ids[req.track_id], self.now_s()
                res.accepted, res.message = True, f'pursuing track {req.track_id}'
        else:
            res.accepted, res.message = self.active, 'stopping pursuit; hover then position hold' if self.active else 'not active'
        self.log.emit(res.request_id, cmd, 'ACCEPTED' if res.accepted else 'REJECTED', res.message, req.track_id)
        if res.accepted and req.engage:
            self._engage(req.track_id, res.request_id)
        elif res.accepted:
            self._release('CANCELED', 'stopped by operator')
        return res

    def _engage(self, tid, rid):
        self.track_id, self.req, self.active, self.state = tid, rid, True, 'CLOSING'
        self.cmd_v[:] = 0
        self.terminal = False
        self.prev_range = None
        self.prev_det_stamp = None
        self.closing = None
        self.t0 = self.now_s()
        d = os.path.join(os.environ.get('TINS8_LOG_DIR', os.path.expanduser('~/.ros/tins8')), 'pursuit')
        os.makedirs(d, exist_ok=True)
        stamp = time.strftime('%Y%m%d-%H%M%S')
        self.csv_path = os.path.join(d, f'{self.uav}-track{tid}-{stamp}.csv')
        self.csv = open(self.csv_path, 'w', newline='')
        self.writer = csv.writer(self.csv)
        self.writer.writerow(['t_s', 'state', 'range_est_m', 'h_frac', 'bearing_deg', 'elev_err_deg', 'err_u_px', 'err_v_px',
                              'bbox_in_fov', 'cmd_vx', 'cmd_vy', 'cmd_vz', 'cmd_yawspeed', 'speed_mps', 'pitch_deg', 'alt_m'])
        self.metrics = {'uav': self.uav, 'track_id': tid, 'engaged_sim_time_s': self.t0, 'time_to_intercept_s': None,
                        'peak_closing_speed_est_mps': 0.0, 'peak_ground_speed_mps': 0.0, 'samples': 0,
                        'in_fov_samples': 0, 'abs_err_u_px': [], 'min_alt_m': 1e9, 'max_pitch_deg': 0.0,
                        'terminal_range_est_m': None, 'lost_events': 0, 'outcome': None, 'csv': os.path.basename(self.csv_path)}
        self.pub_owner.publish(String(data='pursuit'))

    def _release(self, state, detail):
        if not self.active:
            return
        self.active = False
        self.state = 'IDLE'
        self._stream(np.zeros(3), 0.0)         # one zero-velocity setpoint, then the agent holds position
        self.pub_owner.publish(String(data='agent'))
        if self.metrics:
            m = self.metrics
            errs = np.array(m.pop('abs_err_u_px') or [0.0])
            m.update({'outcome': state, 'detail': detail, 'mean_abs_err_u_px': float(errs.mean()),
                      'p95_abs_err_u_px': float(np.percentile(errs, 95)),
                      'fov_containment': (m['in_fov_samples'] / m['samples']) if m['samples'] else None,
                      'duration_s': self.now_s() - self.t0})
            self.csv.close()
            with open(self.csv_path.replace('.csv', '.json'), 'w') as f:
                json.dump(m, f, indent=2)
            self.get_logger().info(f'pursuit summary: {json.dumps(m)}')
        self.log.emit(self.req or '-', 'ENGAGE', state, detail, self.track_id)
        self.track_id = -1

    # ------------------------------------------------------------- loop
    def tick(self):
        self.pub_state.publish(String(data=f'{self.state} track={self.track_id}'))
        if not self.active:
            return
        now = self.now_s()
        dt = 1.0 / self.get_parameter('rate_hz').value
        r = quat_to_matrix(self.att.q)
        _, pitch, _ = euler_from_matrix(r)
        alt = -self.lpos.z + (self.lpos.ref_alt - scenario_alt(self))
        seen_age = now - (self.det_seen_t or 0)
        target = np.zeros(3)
        yawspeed = 0.0
        row = None
        if seen_age <= self.get_parameter('lost_after_s').value and self.det is not None:
            if self.state in ('LOST', 'SEARCH'):
                self.state = 'CLOSING'
            k = self.cam.k
            fx, fy, cx, cy = k[0], k[4], k[2], k[5]
            b = self.det.bbox
            u, v, h_px = b.center.position.x, b.center.position.y, b.size_y
            # Latency compensation: the box is from an image rendered det_stamp; use the attitude
            # of that moment for the ray, then express the bearing against the CURRENT heading.
            # Without this, the yaw loop keeps turning on a stale bearing and overshoots.
            q_img = self.att_hist.at(self.det_stamp) if self.det_stamp is not None else None
            q_img = q_img if q_img is not None else self.att.q
            ray = pixel_to_level_ray(u, v, fx, fy, cx, cy, self.p.camera_tilt_rad, q_img)
            bearing_img, elev = bearing_elevation(ray)
            yaw_img = euler_from_matrix(quat_to_matrix(q_img))[0]
            yaw_now = euler_from_matrix(r)[0]
            bearing = wrap(bearing_img + yaw_img - yaw_now)
            # Measured closing speed: ground velocity (NED) projected on the line of sight.
            los = pixel_to_ned_ray(u, v, fx, fy, cx, cy, self.p.camera_tilt_rad, q_img)
            v_close = float(np.dot([self.lpos.vx, self.lpos.vy, self.lpos.vz], los))
            if self.closing is not None:          # range rate also contains the person's motion
                v_close = max(v_close, self.closing)
            age = max(0.0, now - self.det_stamp) if self.det_stamp is not None else 0.0
            clipped = (v - h_px / 2) <= 3 or (v + h_px / 2) >= self.cam.height - 3
            cmd = ibvs_step(self.p, bearing, elev, h_px, self.cam.height, fy, alt, self.terminal,
                            v_close_meas=v_close, meas_age_s=min(age, 1.0), clipped=clipped)
            if cmd.terminal and not self.terminal and self.metrics['time_to_intercept_s'] is None:
                self.metrics['time_to_intercept_s'] = now - self.t0
                self.metrics['terminal_range_est_m'] = cmd.range_est
                self.log.emit(self.req or '-', 'ENGAGE', 'COMPLETED',
                              f'standoff reached in {now - self.t0:.2f} s (est. range {cmd.range_est:.2f} m); terminal hover', self.track_id)
            self.terminal = cmd.terminal
            self.state = 'TERMINAL' if cmd.terminal else 'CLOSING'
            target = np.array([cmd.vx_fwd, cmd.vy_right, cmd.vz_down])
            yawspeed = cmd.yawspeed
            # Closing speed from consecutive DETECTIONS (range only changes when a new box
            # arrives), smoothed; image stamps give the true time between the two ranges.
            if self.det_stamp is not None and self.det_stamp != self.prev_det_stamp:
                if self.prev_range is not None and self.prev_det_stamp is not None:
                    dts = self.det_stamp - self.prev_det_stamp
                    if dts > 1e-3:
                        c = (self.prev_range - cmd.range_est) / dts
                        self.closing = c if self.closing is None else 0.5 * self.closing + 0.5 * c
                        self.metrics['peak_closing_speed_est_mps'] = max(self.metrics['peak_closing_speed_est_mps'], self.closing)
                self.prev_range, self.prev_det_stamp = cmd.range_est, self.det_stamp
            x1, y1 = u - b.size_x / 2, v - h_px / 2
            in_fov = x1 > 1 and y1 > 1 and x1 + b.size_x < self.cam.width - 1 and y1 + h_px < self.cam.height - 1
            row = [cmd.range_est, h_px / self.cam.height, math.degrees(bearing), math.degrees(elev - self.p.camera_tilt_rad),
                   u - cx, v - cy, int(in_fov)]
            self.metrics['samples'] += 1
            self.metrics['in_fov_samples'] += int(in_fov)
            self.metrics['abs_err_u_px'].append(abs(u - cx))
        else:
            if self.state not in ('LOST', 'SEARCH'):
                self.state = 'LOST'
                self.lost_t = now
                self.metrics['lost_events'] += 1
                self.log.emit(self.req or '-', 'TRACK', 'FAILED', 'selected ID lost > 0.5 s: hover in place', self.track_id)
            if now - self.lost_t > 2.0:
                self.state = 'SEARCH'
                yawspeed = self.get_parameter('search_yaw_rate').value
            if now - self.lost_t > self.get_parameter('search_timeout_s').value:
                self._release('FAILED', 'target not reacquired; position hold')
                return
        step = self.get_parameter('max_accel').value * dt
        self.cmd_v = np.array([slew(self.cmd_v[i], target[i], step) for i in range(3)])
        self._stream(self.cmd_v, yawspeed)
        speed = math.hypot(self.lpos.vx, self.lpos.vy)
        self.metrics['peak_ground_speed_mps'] = max(self.metrics['peak_ground_speed_mps'], speed)
        self.metrics['min_alt_m'] = min(self.metrics['min_alt_m'], alt)
        self.metrics['max_pitch_deg'] = max(self.metrics['max_pitch_deg'], abs(math.degrees(pitch)))
        r_ = row or [None] * 7
        self.writer.writerow([f'{now - self.t0:.3f}', self.state] + r_ + [*(f'{v:.3f}' for v in self.cmd_v),
                             f'{yawspeed:.3f}', f'{speed:.3f}', f'{math.degrees(pitch):.2f}', f'{alt:.2f}'])

    def _stream(self, v_level, yawspeed):
        """Level-frame (fwd, right, down) velocity -> NED velocity setpoint."""
        now_us = int(self.get_clock().now().nanoseconds / 1000)
        mode = OffboardControlMode()
        mode.timestamp = now_us
        mode.velocity = True
        self.pub_mode.publish(mode)
        yaw = euler_from_matrix(quat_to_matrix(self.att.q))[0] if self.att else 0.0
        c, s = math.cos(yaw), math.sin(yaw)
        vn = c * v_level[0] - s * v_level[1]
        ve = s * v_level[0] + c * v_level[1]
        sp = TrajectorySetpoint()
        sp.timestamp = now_us
        sp.position = [math.nan] * 3
        sp.velocity = [float(vn), float(ve), float(v_level[2])]
        sp.acceleration = [math.nan] * 3
        sp.yaw = math.nan
        sp.yawspeed = float(yawspeed)
        self.pub_sp.publish(sp)


def scenario_alt(node):
    if not hasattr(node, '_origin_alt'):
        node._origin_alt = scenario()['world_origin']['alt'] + node.get_parameter('ground_z_enu').value
    return node._origin_alt


def main():
    rclpy.init()
    rclpy.spin(PursuitNode())


if __name__ == '__main__':
    main()
