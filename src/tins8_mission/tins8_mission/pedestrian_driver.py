"""Scripted pedestrian motion (scenario control, clearly a scripted effect).

Each walker is a Gazebo model with the VelocityControl system.  This node closes a simple
unicycle loop on the model's simulator pose to follow the waypoint route in
config/scenario.yaml.  Pedestrian ground truth is used here only to drive the scenario
and by the evaluator; detection never sees it.

  sub /gz/dynamic_poses                    tf2_msgs/TFMessage (Gazebo dynamic_pose/info)
  pub /model/<person>/cmd_vel              geometry_msgs/Twist
  srv /scenario/actor                      tins8_interfaces/ActorCommand  (WALK | STOP | LEAVE | RETURN)
"""
import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from tf2_msgs.msg import TFMessage

from tins8_interfaces.srv import ActorCommand

from .common import CommandLog, new_request_id, scenario
from .geo import wrap


class Walker:
    def __init__(self, name, cfg):
        self.name = name
        self.speed = float(cfg['speed'])
        self.route = [tuple(p) for p in cfg['route']]
        self.exit = [tuple(p) for p in cfg.get('exit', [])]
        self.mode = 'WALK'
        self.path = list(self.route)
        self.i = 1 % len(self.path)
        self.pose = None


class PedestrianDriver(Node):
    def __init__(self):
        super().__init__('pedestrian_driver')
        self.declare_parameter('autostart', True)
        cfg = scenario()
        self.log = CommandLog(self, 'scenario')
        self.walkers = {n: Walker(n, c) for n, c in cfg.get('walkers', {}).items()}
        if not self.get_parameter('autostart').value:
            for w in self.walkers.values():
                w.mode = 'STOP'
        self.pubs = {n: self.create_publisher(Twist, f'/model/{n}/cmd_vel', 10) for n in self.walkers}
        self.create_subscription(TFMessage, '/gz/dynamic_poses', self.on_poses, 10)
        self.create_service(ActorCommand, '/scenario/actor', self.on_command)
        self.create_timer(0.05, self.tick)

    def on_poses(self, msg):
        for t in msg.transforms:
            w = self.walkers.get(t.child_frame_id)
            if w is not None:
                q = t.transform.rotation
                yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
                w.pose = (t.transform.translation.x, t.transform.translation.y, yaw)

    def on_command(self, req, res):
        rid = new_request_id('act')
        cmd = req.command.strip().upper()
        self.log.emit(rid, cmd, 'REQUESTED', argument=req.actor)
        w = self.walkers.get(req.actor)
        if w is None:
            res.accepted, res.message = False, f'{req.actor} is not a walker; walkers: {sorted(self.walkers)}'
        elif cmd == 'WALK':
            w.mode, w.path, w.i = 'WALK', list(w.route), self._nearest(w, w.route)
            res.accepted, res.message = True, 'walking its loop'
        elif cmd == 'STOP':
            w.mode = 'STOP'
            res.accepted, res.message = True, 'standing still'
        elif cmd == 'LEAVE':
            if not w.exit:
                res.accepted, res.message = False, 'no exit path configured'
            else:
                w.mode, w.path, w.i = 'LEAVE', list(w.exit), 0
                res.accepted, res.message = True, f'leaving view along {w.exit}'
        elif cmd == 'RETURN':
            w.mode, w.path, w.i = 'RETURN', list(reversed(w.exit)) + [w.route[0]], 0
            res.accepted, res.message = True, 'returning to its loop'
        else:
            res.accepted, res.message = False, f'unknown command {cmd}'
        self.log.emit(rid, cmd, 'COMPLETED' if res.accepted else 'REJECTED', res.message, req.actor)
        return res

    @staticmethod
    def _nearest(w, path):
        if w.pose is None:
            return 0
        return min(range(len(path)), key=lambda k: math.hypot(path[k][0] - w.pose[0], path[k][1] - w.pose[1]))

    def tick(self):
        for n, w in self.walkers.items():
            cmd = Twist()
            if w.pose is not None and w.mode != 'STOP' and w.i < len(w.path):
                gx, gy = w.path[w.i]
                dx, dy = gx - w.pose[0], gy - w.pose[1]
                if math.hypot(dx, dy) < 0.4:
                    w.i += 1
                    if w.i >= len(w.path):
                        if w.mode == 'WALK':
                            w.i = 0
                        elif w.mode == 'RETURN':
                            w.mode, w.path, w.i = 'WALK', list(w.route), 1 % len(w.route)
                        else:
                            w.mode = 'STOP'
                else:
                    err = wrap(math.atan2(dy, dx) - w.pose[2])
                    cmd.angular.z = max(-1.5, min(1.5, 2.5 * err))
                    cmd.linear.x = w.speed * max(0.0, math.cos(err)) if abs(err) < 1.2 else 0.0
            self.pubs[n].publish(cmd)


def main():
    # Own SIGINT/SIGTERM handling: Gazebo's VelocityControl keeps the last command, so the
    # walkers must be sent a zero velocity before this node goes away.
    import signal
    from rclpy.signals import SignalHandlerOptions
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = PedestrianDriver()
    stop = {'flag': False}
    signal.signal(signal.SIGINT, lambda *_: stop.__setitem__('flag', True))
    signal.signal(signal.SIGTERM, lambda *_: stop.__setitem__('flag', True))
    while rclpy.ok() and not stop['flag']:
        rclpy.spin_once(node, timeout_sec=0.1)
    for _ in range(5):
        for pub in node.pubs.values():
            pub.publish(Twist())
        rclpy.spin_once(node, timeout_sec=0.05)
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
