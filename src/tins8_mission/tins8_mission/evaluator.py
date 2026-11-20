"""Evaluation logger.  Uses simulator GROUND TRUTH, which no control or perception node uses.

Writes $TINS8_LOG_DIR/<run>/ with:
  telemetry.csv   10 Hz: each UAV's PX4-estimated position, commanded hold target, hold error,
                  phase, owner; ground-truth poses of uav01/uav02/walkers/UGV; uav01 track status
  commands.csv    every CommandStatus line (/tins8/command_log)
  fleet.csv       fleet counts whenever they change
The offline script scripts/analyze_run.py turns these into the result tables and plots.
"""
import csv
import math
import os
import time

import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage

from tins8_interfaces.msg import CommandStatus, FleetState, TrackStatus, UavStatus

from .common import LATCHED_QOS, LOG_QOS, UAVS, scenario


class Evaluator(Node):
    def __init__(self):
        super().__init__('evaluator')
        self.declare_parameter('run_name', '')
        name = self.get_parameter('run_name').value or time.strftime('run-%Y%m%d-%H%M%S')
        self.dir = os.path.join(os.environ.get('TINS8_LOG_DIR', os.path.expanduser('~/.ros/tins8')), name)
        os.makedirs(self.dir, exist_ok=True)
        self.walkers = list(scenario().get('walkers', {}))
        self.gt_names = list(UAVS) + self.walkers + ['tins_ugv']
        self.status, self.gt, self.track, self.fleet_key = {}, {}, None, None
        self.f_tel = open(os.path.join(self.dir, 'telemetry.csv'), 'w', newline='')
        self.tel = csv.writer(self.f_tel)
        head = ['t_sim']
        for u in UAVS:
            head += [f'{u}_{k}' for k in ('phase', 'owner', 'connected', 'age_s', 'est_x', 'est_y', 'est_z',
                                          'hold_x', 'hold_y', 'hold_z', 'hold_err', 'target', 'last_cmd', 'last_state')]
        for n in self.gt_names:
            head += [f'gt_{n}_{k}' for k in ('x', 'y', 'z', 'qx', 'qy', 'qz', 'qw')]
        head += ['trk_state', 'trk_id', 'trk_x1', 'trk_y1', 'trk_x2', 'trk_y2', 'trk_err_x', 'trk_err_y',
                 'trk_latency', 'trk_visible']
        self.tel.writerow(head)
        self.f_cmd = open(os.path.join(self.dir, 'commands.csv'), 'w', newline='')
        self.cmd = csv.writer(self.f_cmd)
        self.cmd.writerow(['t_sim', 'request_id', 'target', 'command', 'argument', 'state', 'detail'])
        self.f_fleet = open(os.path.join(self.dir, 'fleet.csv'), 'w', newline='')
        self.fl = csv.writer(self.f_fleet)
        self.fl.writerow(['t_sim', 'stowed', 'active_docked', 'airborne', 'slots'])
        for u in UAVS:
            self.create_subscription(UavStatus, f'/{u}/status', lambda m, u=u: self.status.__setitem__(u, m), 10)
        self.create_subscription(TFMessage, '/gz/dynamic_poses', self.on_gt, 10)
        self.create_subscription(TrackStatus, '/uav01/track/status', lambda m: setattr(self, 'track', m), 10)
        self.create_subscription(CommandStatus, '/tins8/command_log', self.on_cmd, LOG_QOS)
        self.create_subscription(FleetState, '/fleet/state', self.on_fleet, LATCHED_QOS)
        self.create_timer(0.1, self.tick)
        self.get_logger().info(f'logging evaluation data to {self.dir}')

    def t(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_gt(self, m):
        for tr in m.transforms:
            if tr.child_frame_id in self.gt_names:
                self.gt[tr.child_frame_id] = tr.transform

    def on_cmd(self, m):
        self.cmd.writerow([f'{self.t():.3f}', m.request_id, m.target, m.command, m.argument, m.state, m.detail])
        self.f_cmd.flush()

    def on_fleet(self, m):
        key = (m.stowed, m.active_docked, m.airborne, tuple(s.state for s in m.slots))
        if key != self.fleet_key:
            self.fleet_key = key
            self.fl.writerow([f'{self.t():.3f}', m.stowed, m.active_docked, m.airborne,
                              ' '.join(f'{s.uav_id}:{s.state}:{s.level}' for s in m.slots)])
            self.f_fleet.flush()

    def tick(self):
        row = [f'{self.t():.3f}']
        for u in UAVS:
            s = self.status.get(u)
            if s is None:
                row += [''] * 14
                continue
            row += [s.phase, s.control_owner, int(s.connected), f'{s.telemetry_age_s:.3f}',
                    f'{s.position_enu.x:.3f}', f'{s.position_enu.y:.3f}', f'{s.position_enu.z:.3f}',
                    f'{s.hold_target_enu.x:.3f}', f'{s.hold_target_enu.y:.3f}', f'{s.hold_target_enu.z:.3f}',
                    f'{s.hold_error_m:.3f}', s.target_name, s.last_command, s.last_command_state]
        for n in self.gt_names:
            g = self.gt.get(n)
            if g is None:
                row += [''] * 7
            else:
                row += [f'{v:.4f}' for v in (g.translation.x, g.translation.y, g.translation.z,
                                             g.rotation.x, g.rotation.y, g.rotation.z, g.rotation.w)]
        k = self.track
        if k is None:
            row += [''] * 10
        else:
            b = list(k.bbox_xyxy) + [math.nan] * (4 - len(k.bbox_xyxy))
            row += [k.state, k.selected_id] + [f'{v:.1f}' for v in b] + [f'{k.center_error_px_x:.1f}', f'{k.center_error_px_y:.1f}',
                                                                         f'{k.selection_to_centered_s:.3f}', ' '.join(map(str, k.visible_person_ids))]
        self.tel.writerow(row)
        if int(self.t() * 10) % 20 == 0:
            self.f_tel.flush()


def main():
    rclpy.init()
    node = Evaluator()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        for f in (node.f_tel, node.f_cmd, node.f_fleet):
            f.close()


if __name__ == '__main__':
    main()
