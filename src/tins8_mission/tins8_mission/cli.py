"""tins8ctl: terminal control for every assessed operation (same ROS interfaces as the GUI).

  tins8ctl status                       one snapshot of fleet, carousel, both UAVs and the tracker
  tins8ctl watch                        stream the shared command log
  tins8ctl points                       list observation points and inspection waypoints
  tins8ctl carousel release uav01|uav02 | index-b | index-a | stop
  tins8ctl uav uav01 takeoff | obs op1 | goto wp_bus_stop | hold | land [here|pad_west|ugv|restow]
  tins8ctl select 3 | clear             choose the YOLOX track shown in the uav01 inset
  tins8ctl pursuit uav01 start 3 | stop high-speed IBVS pursuit (separate mode)
  tins8ctl actor person_12 walk|stop|leave|return
  tins8ctl mission start [--uav1 op1] [--uav2 op2] [--track N]   (Ctrl+C or 'mission cancel' cancels)
  tins8ctl mission cancel
  tins8ctl teleop                       keyboard teleoperation of the UGV (/ugv/cmd_vel)
  tins8ctl shutdown                     stop every component started by the launch scripts
Add --wait to a command to block until it reaches COMPLETED / FAILED / CANCELED / REJECTED.
"""
import argparse
import os
import subprocess
import sys
import time

import rclpy
from action_msgs.srv import CancelGoal
from rclpy.action import ActionClient
from rclpy.node import Node

from tins8_interfaces.action import RunMission
from tins8_interfaces.msg import CarouselState, CommandStatus, FleetState, TrackStatus, UavStatus
from tins8_interfaces.srv import ActorCommand, CarouselCommand, PursuitCommand, SelectTarget, UavCommand

from .common import LATCHED_QOS, LOG_QOS, UAVS, scenario

TERMINAL = ('COMPLETED', 'FAILED', 'CANCELED', 'REJECTED')


class Ctl(Node):
    def __init__(self):
        super().__init__('tins8ctl')
        self.log_lines = []
        self.create_subscription(CommandStatus, '/tins8/command_log', self.log_lines.append, LOG_QOS)

    def call(self, srv_type, name, req, timeout=5.0):
        cli = self.create_client(srv_type, name)
        if not cli.wait_for_service(timeout_sec=timeout):
            print(f'service {name} is not available (is the simulation running?)')
            sys.exit(2)
        fut = cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout)
        if not fut.done():
            print(f'no response from {name}')
            sys.exit(3)
        return fut.result()

    def wait_request(self, rid, timeout):
        t = time.monotonic()
        seen = 0
        while time.monotonic() - t < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
            for m in self.log_lines[seen:]:
                if m.request_id == rid:
                    print(f'  {m.target} {m.command} -> {m.state}  {m.detail}')
                    if m.state in TERMINAL:
                        return m.state
            seen = len(self.log_lines)
        print('  (still running; follow it with: tins8ctl watch)')
        return None

    def snapshot(self, msg_type, topic, qos=10, timeout=4.0):   # 4 s: discovery + sim-time rates under load
        box = []
        sub = self.create_subscription(msg_type, topic, box.append, qos)
        t = time.monotonic()
        while not box and time.monotonic() - t < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
        self.destroy_subscription(sub)
        return box[-1] if box else None


def report(res, ctl, args):
    ok = getattr(res, 'accepted', False)
    print(f"{'ACCEPTED' if ok else 'REJECTED'}: {res.message}" + (f'  [{res.request_id}]' if getattr(res, 'request_id', '') else ''))
    if ok and args.wait and getattr(res, 'request_id', ''):
        ctl.wait_request(res.request_id, args.timeout)
    return 0 if ok else 1


def cmd_status(ctl):
    f = ctl.snapshot(FleetState, '/fleet/state', LATCHED_QOS)
    if f:
        print(f'FLEET  stowed={f.stowed} active_docked={f.active_docked} airborne={f.airborne}  (8 positions)')
        for s in f.slots:
            print(f'  {s.uav_id}  {s.tray}/{s.level:5s}  {s.state:13s} {"independent PX4 model" if s.independent_model else "stowed visual"}')
    else:
        print('FLEET  no /fleet/state')
    c = ctl.snapshot(CarouselState, '/carousel/state', LATCHED_QOS)
    if c:
        q = ' '.join(f'{n.replace("_joint", "")}={v * 1000:+.1f}mm' for n, v in zip(c.joint_names, c.measured_position_m))
        print(f'CAROUSEL  station={c.station} activity={c.activity} fresh={c.measured_fresh} latched={list(c.latched)}\n  measured {q}')
    for u in UAVS:
        s = ctl.snapshot(UavStatus, f'/{u}/status')
        if s:
            p = s.position_enu
            print(f'{u.upper()}  {s.phase:8s} owner={s.control_owner:7s} armed={s.armed} nav={s.nav_state:9s} '
                  f'telemetry_age={s.telemetry_age_s:.2f}s  pos=({p.x:.1f},{p.y:.1f},{p.z:.1f})  target={s.target_name or "-"} '
                  f'err={s.hold_error_m:.2f}m  last="{s.last_command}" {s.last_command_state}')
        else:
            print(f'{u.upper()}  no status (agent not running?)')
    k = ctl.snapshot(TrackStatus, '/uav01/track/status')
    if k:
        print(f'TRACKER  {k.state} selected={k.selected_id} visible_ids={list(k.visible_person_ids)} '
              f'centre_err=({k.center_error_px_x:.0f},{k.center_error_px_y:.0f})px latency={k.selection_to_centered_s:.2f}s')
    return 0


def cmd_watch(ctl):
    print('command log (Ctrl+C to stop)')
    seen = 0
    try:
        while True:
            rclpy.spin_once(ctl, timeout_sec=0.2)
            for m in ctl.log_lines[seen:]:
                print(f'{m.stamp.sec:>7d}.{m.stamp.nanosec // 10**7:02d}  {m.target:12s} {m.command:18s} {m.argument:12s} {m.state:10s} {m.detail}')
            seen = len(ctl.log_lines)
    except KeyboardInterrupt:
        return 0


def cmd_mission(ctl, args):
    if args.action == 'cancel':
        res = ctl.call(CancelGoal, '/mission/run/_action/cancel_goal', CancelGoal.Request())
        print(f'cancel request sent; {len(res.goals_canceling)} goal(s) canceling')
        return 0
    client = ActionClient(ctl, RunMission, '/mission/run')
    if not client.wait_for_server(timeout_sec=5):
        print('mission server not available')
        return 2
    goal = RunMission.Goal(uav1_point=args.uav1, uav2_point=args.uav2, track_id=args.track)
    print(f'REQUESTED mission: uav01->{args.uav1} uav02->{args.uav2} track={args.track}')
    fut = client.send_goal_async(goal, feedback_callback=lambda f: print(
        f'  [{f.feedback.progress * 100:5.1f}%] {f.feedback.phase:10s} {f.feedback.detail}'))
    rclpy.spin_until_future_complete(ctl, fut)
    handle = fut.result()
    if not handle.accepted:
        print('REJECTED (another mission is running?)')
        return 1
    print('ACCEPTED  (Ctrl+C cancels the mission)')
    res_fut = handle.get_result_async()
    try:
        rclpy.spin_until_future_complete(ctl, res_fut)
    except KeyboardInterrupt:
        print('cancel requested...')
        cf = handle.cancel_goal_async()
        rclpy.spin_until_future_complete(ctl, cf)
        rclpy.spin_until_future_complete(ctl, res_fut, timeout_sec=10)
    r = res_fut.result().result if res_fut.done() else None
    if r:
        print(f'{r.outcome}: {r.summary}')
    return 0 if r and r.success else 1


def main(argv=None):
    ap = argparse.ArgumentParser(prog='tins8ctl', description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--wait', action='store_true', help='block until the request finishes')
    ap.add_argument('--timeout', type=float, default=180.0)
    sub = ap.add_subparsers(dest='cmd', required=True)
    sub.add_parser('status')
    sub.add_parser('watch')
    sub.add_parser('points')
    c = sub.add_parser('carousel')
    c.add_argument('action', choices=['release', 'index-b', 'index-a', 'stop'])
    c.add_argument('uav', nargs='?', default='')
    u = sub.add_parser('uav')
    u.add_argument('uav', choices=list(UAVS))
    u.add_argument('action', choices=['takeoff', 'obs', 'goto', 'hold', 'land'])
    u.add_argument('point', nargs='?', default='')
    s = sub.add_parser('select')
    s.add_argument('track')
    p = sub.add_parser('pursuit')
    p.add_argument('uav', choices=list(UAVS))
    p.add_argument('action', choices=['start', 'stop'])
    p.add_argument('track', nargs='?', type=int, default=-1)
    a = sub.add_parser('actor')
    a.add_argument('name')
    a.add_argument('action', choices=['walk', 'stop', 'leave', 'return'])
    m = sub.add_parser('mission')
    m.add_argument('action', choices=['start', 'cancel'])
    m.add_argument('--uav1', default='op1')
    m.add_argument('--uav2', default='op2')
    m.add_argument('--track', type=int, default=-1)
    sub.add_parser('teleop')
    sub.add_parser('shutdown')
    args = ap.parse_args(argv)

    if args.cmd == 'teleop':
        os.execvp('ros2', ['ros2', 'run', 'teleop_twist_keyboard', 'teleop_twist_keyboard',
                           '--ros-args', '-r', 'cmd_vel:=/ugv/cmd_vel'])
    if args.cmd == 'shutdown':
        script = os.path.join(os.environ.get('TINS8_WS', '.'), 'scripts', 'stop_all.sh')
        return subprocess.call(['bash', script])
    if args.cmd == 'points':
        cfg = scenario()
        for k in ('observation_points', 'inspection_waypoints'):
            print(k)
            for n, p in cfg[k].items():
                print(f"  {n:16s} x={p['x']:7.1f} y={p['y']:7.1f} z={p['z']:5.1f} yaw={p.get('yaw_deg', 0):6.1f} deg")
        print('landing_pads  (land here | <pad> implemented; land ugv | restow are rejected with the reason)')
        for n, p in cfg.get('landing_pads', {}).items():
            print(f"  {n:16s} x={p['x']:7.1f} y={p['y']:7.1f}")
        return 0

    rclpy.init()
    ctl = Ctl()
    try:
        if args.cmd == 'status':
            return cmd_status(ctl)
        if args.cmd == 'watch':
            return cmd_watch(ctl)
        if args.cmd == 'mission':
            return cmd_mission(ctl, args)
        if args.cmd == 'carousel':
            command = {'release': 'RELEASE', 'index-b': 'INDEX_TO_B_UPPER', 'index-a': 'INDEX_TO_A_UPPER', 'stop': 'STOP'}[args.action]
            res = ctl.call(CarouselCommand, '/carousel/command', CarouselCommand.Request(command=command, argument=args.uav))
            return report(res, ctl, args)
        if args.cmd == 'uav':
            command = {'takeoff': 'TAKEOFF_HOLD', 'obs': 'GOTO_OBSERVATION', 'goto': 'GOTO_WAYPOINT', 'hold': 'HOLD', 'land': 'LAND'}[args.action]
            res = ctl.call(UavCommand, f'/{args.uav}/command', UavCommand.Request(uav_id=args.uav, command=command, argument=args.point))
            return report(res, ctl, args)
        if args.cmd == 'select':
            tid = -1 if args.track == 'clear' else int(args.track)
            res = ctl.call(SelectTarget, '/uav01/tracker/select', SelectTarget.Request(track_id=tid))
            return report(res, ctl, args)
        if args.cmd == 'pursuit':
            res = ctl.call(PursuitCommand, f'/{args.uav}/pursuit/command',
                           PursuitCommand.Request(engage=args.action == 'start', track_id=args.track))
            return report(res, ctl, args)
        if args.cmd == 'actor':
            res = ctl.call(ActorCommand, '/scenario/actor', ActorCommand.Request(actor=args.name, command=args.action.upper()))
            print(f"{'ACCEPTED' if res.accepted else 'REJECTED'}: {res.message}")
            return 0 if res.accepted else 1
    finally:
        ctl.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    sys.exit(main())
