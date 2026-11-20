"""Shared ROS helpers: scenario config, QoS profiles and the command log."""
import collections
import itertools
import os
import time

import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from tins8_interfaces.msg import CommandStatus

UAVS = ('uav01', 'uav02')
ALL_SLOTS = tuple(f'uav{i:02d}' for i in range(1, 9))
TRAY_OF = {u: ('tray_a' if int(u[-1]) % 2 else 'tray_b') for u in ALL_SLOTS}

# PX4 publishes /fmu/out/* best effort; subscribers must match.
PX4_OUT_QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         history=HistoryPolicy.KEEP_LAST, depth=1)
LATCHED_QOS = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         history=HistoryPolicy.KEEP_LAST, depth=1)
LOG_QOS = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE,
                     history=HistoryPolicy.KEEP_LAST, depth=100)


def px4_topic(ns, direction, name, msg_type):
    """PX4 topic name with the uORB message-version suffix (PX4 >= 1.16 publishes e.g. vehicle_status_v1)."""
    v = getattr(msg_type, 'MESSAGE_VERSION', 0) or 0
    return f'{ns}/fmu/{direction}/{name}' + (f'_v{v}' if v > 0 else '')


class AttitudeHistory:
    """Short buffer of (time_s, PX4 quaternion) so an image can be paired with the attitude
    the vehicle had when the frame was rendered (detections arrive 0.1-0.5 s later)."""

    def __init__(self, span_s=3.0):
        self.span = span_s
        self.buf = collections.deque()

    def add(self, t, q):
        self.buf.append((t, tuple(float(v) for v in q)))
        while self.buf and t - self.buf[0][0] > self.span:
            self.buf.popleft()

    def at(self, t):
        """Quaternion nearest to time t (None if empty)."""
        if not self.buf:
            return None
        return min(self.buf, key=lambda e: abs(e[0] - t))[1]


def scenario():
    path = os.path.join(get_package_share_directory('tins8_gazebo'), 'config', 'scenario.yaml')
    return yaml.safe_load(open(path))


def mission_params():
    path = os.path.join(get_package_share_directory('tins8_mission'), 'config', 'mission_params.yaml')
    return yaml.safe_load(open(path))


def named_point(cfg, name):
    """Look up an observation point or inspection waypoint by name."""
    for key in ('observation_points', 'inspection_waypoints'):
        if name in cfg.get(key, {}):
            return cfg[key][name]
    return None


_counter = itertools.count(1)


def new_request_id(prefix):
    return f'{prefix}-{int(time.time()) % 100000:05d}-{next(_counter):03d}'


class CommandLog:
    """Publishes CommandStatus lines on /tins8/command_log and mirrors them to the node log."""

    def __init__(self, node, target):
        self.node = node
        self.target = target
        self.pub = node.create_publisher(CommandStatus, '/tins8/command_log', LOG_QOS)

    def emit(self, request_id, command, state, detail='', argument='', target=None):
        m = CommandStatus()
        m.stamp = self.node.get_clock().now().to_msg()
        m.request_id = request_id
        m.target = target or self.target
        m.command = command
        m.argument = str(argument)
        m.state = state
        m.detail = detail
        self.pub.publish(m)
        self.node.get_logger().info(f'[{m.target}] {command} {argument} -> {state} {detail} ({request_id})')
        return m
