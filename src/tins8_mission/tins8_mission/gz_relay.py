"""Gazebo -> ROS 2 relay for the two things the stock ros_gz_bridge cannot carry correctly.

1. Ground-truth poses with entity names
     sub (Gazebo)  /world/<world>/dynamic_pose/info   gz.msgs.Pose_V
     pub (ROS 2)   /gz/dynamic_poses                  tf2_msgs/TFMessage, child_frame_id = entity name
   The stock Pose_V -> TFMessage conversion leaves child_frame_id empty, so the names
   (uav01, person_12, tins_ugv, ...) would be lost.  Ground truth is never used by the
   controllers; only the evaluator, the pedestrian driver and the fleet markers read it.

2. Dock latch state, latched
     sub (Gazebo)  /ugv/latch/<uav>/state             gz.msgs.StringMsg  ("attached" | "detached")
     pub (ROS 2)   /ugv/latch/<uav>/state             std_msgs/String, TRANSIENT_LOCAL
   The DetachableJoint system publishes its state only when it changes (the first "attached"
   comes at the first physics step).  This node is started just before the Gazebo server so it
   is subscribed by then, and it re-offers the last state to late ROS subscribers.

3. Simulation clock, at a fixed resolution
     sub (Gazebo)  /clock                             gz.msgs.Clock  (every physics step, 250 Hz)
     pub (ROS 2)   /clock                             rosgraph_msgs/Clock, every clock_resolution_s
   Every node that uses sim time wakes up on each /clock message.  Forwarding every 4 ms step
   cost each Python node ~30 % of a CPU core; 10 ms is ample for the 30 Hz control loops
   (PX4 itself takes its time directly from Gazebo and is not affected).

All three use the gz-transport Python bindings that ship with ROS 2 Jazzy (gz_transport_vendor).
This node runs on wall time (it is the sim-time source); its messages carry simulator stamps.
"""
import threading

import rclpy
from builtin_interfaces.msg import Time
from geometry_msgs.msg import TransformStamped
from rosgraph_msgs.msg import Clock
from rclpy.node import Node
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage

from gz.msgs10.clock_pb2 import Clock as GzClock
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.msgs10.stringmsg_pb2 import StringMsg
from gz.transport13 import Node as GzNode

from .common import LATCHED_QOS, UAVS


class GzRelay(Node):
    def __init__(self):
        super().__init__('gz_relay')
        self.declare_parameter('world', 'fieldlab')
        self.declare_parameter('rate_hz', 30.0)
        self.declare_parameter('clock_resolution_s', 0.01)
        world = self.get_parameter('world').value
        self.pub = self.create_publisher(TFMessage, '/gz/dynamic_poses', 10)
        self.lock = threading.Lock()
        self.poses = {}              # name -> latest gz Pose (messages may carry only a subset)
        self.fresh = False
        self.gz = GzNode()
        topic = f'/world/{world}/dynamic_pose/info'
        if not self.gz.subscribe(Pose_V, topic, self.on_gz):
            raise RuntimeError(f'cannot subscribe to {topic}')
        self.create_timer(1.0 / self.get_parameter('rate_hz').value, self.tick)
        self.get_logger().info(f'republishing {topic} with entity names on /gz/dynamic_poses')
        self.latch_pub, self.latch_new = {}, {}
        for uav in UAVS:
            t = f'/ugv/latch/{uav}/state'
            self.latch_pub[uav] = self.create_publisher(String, t, LATCHED_QOS)
            self.gz.subscribe(StringMsg, t, lambda m, uav=uav: self.on_latch(uav, m))
        self.create_timer(0.05, self.flush_latch)
        self.sim_ns = 0
        self.clock_last_ns = -1
        self.clock_res_ns = int(self.get_parameter('clock_resolution_s').value * 1e9)
        self.clock_pub = self.create_publisher(Clock, '/clock', 10)
        self.gz.subscribe(GzClock, '/clock', self.on_clock)

    def on_clock(self, msg):         # gz-transport thread; published directly (rclpy publish is thread safe)
        ns = msg.sim.sec * 1_000_000_000 + msg.sim.nsec
        self.sim_ns = ns
        if ns < self.clock_last_ns:          # world reset
            self.clock_last_ns = -1
        if self.clock_last_ns < 0 or ns - self.clock_last_ns >= self.clock_res_ns:
            self.clock_last_ns = ns
            self.clock_pub.publish(Clock(clock=Time(sec=msg.sim.sec, nanosec=msg.sim.nsec)))

    def sim_stamp(self):
        ns = self.sim_ns
        return Time(sec=ns // 1_000_000_000, nanosec=ns % 1_000_000_000)

    def on_latch(self, uav, msg):    # gz-transport thread
        with self.lock:
            self.latch_new[uav] = str(msg.data)

    def flush_latch(self):
        with self.lock:
            new, self.latch_new = self.latch_new, {}
        for uav, state in new.items():
            self.latch_pub[uav].publish(String(data=state))
            self.get_logger().info(f'{uav} latch: {state}')

    def on_gz(self, msg):            # gz-transport thread
        with self.lock:
            for p in msg.pose:
                if p.name:      # copy the values; the binding may reuse the message buffer
                    q, o = p.position, p.orientation
                    self.poses[p.name] = (p.name, q.x, q.y, q.z, o.x, o.y, o.z, o.w)
            self.fresh = True

    def tick(self):
        with self.lock:
            if not self.fresh:
                return
            poses, self.fresh = list(self.poses.values()), False
        out = TFMessage()
        stamp = self.sim_stamp()
        for name, x, y, z, qx, qy, qz, qw in poses:
            t = TransformStamped()
            t.header.stamp = stamp
            t.header.frame_id = 'world'
            t.child_frame_id = name
            tr, ro = t.transform.translation, t.transform.rotation
            tr.x, tr.y, tr.z = x, y, z
            ro.x, ro.y, ro.z, ro.w = qx, qy, qz, qw
            out.transforms.append(t)
        self.pub.publish(out)


def main():
    rclpy.init()
    node = GzRelay()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
