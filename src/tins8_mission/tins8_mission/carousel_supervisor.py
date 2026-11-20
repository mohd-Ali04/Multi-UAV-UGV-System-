"""Carousel supervisor: guarded slide exchange and dock latches, gated on measured joints.

Interfaces
  sub  /ugv/joint_states               sensor_msgs/JointState  (Gazebo JointStatePublisher, measured)
  sub  /ugv/latch/<uav>/state          std_msgs/String         (DetachableJoint output: attached|detached)
  sub  /<uav>/status                   tins8_interfaces/UavStatus (release interlock)
  pub  /ugv/carousel/<joint>/cmd_pos   std_msgs/Float64        (JointPositionController target)
  pub  /ugv/latch/<uav>/detach         std_msgs/Empty
  pub  /carousel/state                 tins8_interfaces/CarouselState
  srv  /carousel/command               tins8_interfaces/CarouselCommand
"""
import os

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Empty, Float64, String

from tins8_interfaces.msg import CarouselState, UavStatus
from tins8_interfaces.srv import CarouselCommand

from .common import UAVS, CommandLog, LATCHED_QOS, new_request_id
from .guarded_exchange import (A_UPPER_INDEX, B_UPPER_INDEX, JOINTS, Sequencer, load_waypoints, reached,
                               route, station_of)


class CarouselSupervisor(Node):
    def __init__(self):
        super().__init__('carousel_supervisor')
        self.declare_parameter('tolerance_m', 0.0015)
        self.declare_parameter('fresh_s', 0.5)
        self.declare_parameter('stall_s', 15.0)
        self.declare_parameter('airborne_clearance_m', 2.0)
        self.tol = self.get_parameter('tolerance_m').value
        path = os.path.join(get_package_share_directory('tins_description'), 'config', 'guarded_exchange_waypoints.yaml')
        self.w = load_waypoints(path)
        self.seq = Sequencer(self.w, self.tol)
        self.log = CommandLog(self, 'carousel')
        self.measured = None
        self.measured_t = None
        self.cmd = list(self.w[A_UPPER_INDEX].q)
        self.activity = 'IDLE'
        self.paused_goal = None
        self.goal = None
        self.active_req = None
        self.last_progress = None
        self.last_index = A_UPPER_INDEX
        self.latch = {u: 'unknown' for u in UAVS}
        self.uav_status = {}
        self.release_req = {}
        self.cmd_pubs = {j: self.create_publisher(Float64, f'/ugv/carousel/{j}/cmd_pos', 10) for j in JOINTS}
        self.detach_pubs = {u: self.create_publisher(Empty, f'/ugv/latch/{u}/detach', 10) for u in UAVS}
        for u in UAVS:
            self.create_subscription(String, f'/ugv/latch/{u}/state', lambda m, u=u: self.on_latch(u, m), LATCHED_QOS)
            self.create_subscription(UavStatus, f'/{u}/status', lambda m, u=u: self.uav_status.__setitem__(u, m), 10)
        self.create_subscription(JointState, '/ugv/joint_states', self.on_joints, 20)
        self.state_pub = self.create_publisher(CarouselState, '/carousel/state', LATCHED_QOS)
        self.create_service(CarouselCommand, '/carousel/command', self.on_command)
        self.create_timer(0.1, self.tick)
        self.create_timer(0.2, self.republish)

    # ---------------------------------------------------------------- inputs
    def on_joints(self, msg):
        pos = dict(zip(msg.name, msg.position))
        if all(j in pos for j in JOINTS):
            self.measured = [pos[j] for j in JOINTS]
            self.measured_t = self.get_clock().now()

    def on_latch(self, uav, msg):
        prev = self.latch.get(uav)
        self.latch[uav] = msg.data
        if msg.data == 'detached' and uav in self.release_req:
            req = self.release_req.pop(uav)
            self.activity = 'IDLE'
            self.log.emit(req, 'RELEASE', 'COMPLETED', 'latch state measured detached', uav)
        elif prev != msg.data:
            self.get_logger().info(f'{uav} latch: {msg.data}')

    def fresh(self):
        if self.measured_t is None:
            return False
        return (self.get_clock().now() - self.measured_t).nanoseconds * 1e-9 <= self.get_parameter('fresh_s').value

    def station(self):
        if self.measured is None:
            return 'UNKNOWN', -1
        return station_of(self.measured, self.w, self.tol)

    # -------------------------------------------------------------- commands
    def on_command(self, req, res):
        cmd = req.command.strip().upper()
        res.request_id = new_request_id('car')
        self.log.emit(res.request_id, cmd, 'REQUESTED', '', req.argument)
        ok, why = self._dispatch(cmd, req.argument.strip().lower(), res.request_id)
        res.accepted, res.message = ok, why
        self.log.emit(res.request_id, cmd, 'ACCEPTED' if ok else 'REJECTED', why, req.argument)
        return res

    def _dispatch(self, cmd, arg, rid):
        if not self.fresh():
            return False, 'no fresh measured joint state'
        station, index = self.station()
        if cmd == 'STOP':
            if self.activity != 'INDEXING':
                return True, 'already stationary'
            self.paused_goal = self.goal
            self.seq.stop()
            self.cmd = list(self.measured)
            self.activity = 'STOPPED'
            if self.active_req:
                self.log.emit(self.active_req, 'INDEX', 'CANCELED', 'stopped by operator at measured position')
                self.active_req = None
            return True, 'holding measured position'
        if cmd == 'RELEASE':
            if arg not in UAVS:
                return False, f'unknown active aircraft {arg!r}'
            if self.activity == 'INDEXING':
                return False, 'carousel is moving'
            access = self.w[index].accessible if index >= 0 else ()
            if f'{arg}_dock' not in access:
                return False, f'{arg}_dock is not accessible at station {station}'
            if self.latch.get(arg) != 'attached':
                return False, f'latch state is {self.latch.get(arg)}'
            self.release_req[arg] = rid
            self.activity = 'RELEASING'
            self.detach_pubs[arg].publish(Empty())
            return True, f'detach sent; waiting for measured latch state'
        if cmd in ('INDEX_TO_B_UPPER', 'INDEX_TO_A_UPPER'):
            goal = B_UPPER_INDEX if cmd.endswith('B_UPPER') else A_UPPER_INDEX
            blocker = self._release_blocker()
            if blocker:
                return False, blocker
            if self.activity == 'INDEXING':
                return False, 'already indexing'
            if self.activity in ('STOPPED', 'FAULT') and self.paused_goal == goal:
                indices = self._resume_route(goal)
            else:
                if index < 0:
                    return False, 'not at an accessible station; use STOP/resume with the paused goal'
                if index == goal:
                    self.log.emit(rid, cmd, 'COMPLETED', f'already at {station}')
                    return True, 'already there'
                indices = route(index, goal)
            self.goal = goal
            self.active_req = rid
            self.cmd = list(self.seq.start(indices))
            self.activity = 'INDEXING'
            self.last_progress = self.get_clock().now()
            return True, f'guarded route through waypoints {indices[0]}..{indices[-1]}'
        return False, f'unknown command {cmd}'

    def _resume_route(self, goal):
        # Resume from the next waypoint after the last measured one on this route.
        full = route(A_UPPER_INDEX, B_UPPER_INDEX) if goal == B_UPPER_INDEX else route(B_UPPER_INDEX, A_UPPER_INDEX)
        rest = full[full.index(self.last_index) + 1:] if self.last_index in full else full
        return rest or full[-1:]

    def _release_blocker(self):
        """Do not move the trays while a released aircraft may still be on or near its dock."""
        clearance = self.get_parameter('airborne_clearance_m').value
        for u in UAVS:
            if self.latch.get(u) == 'detached':
                st = self.uav_status.get(u)
                if st is None or not st.connected:
                    return f'{u} is released but its telemetry is not available'
                if st.position_enu.z < clearance or st.phase in ('DOCKED', 'LANDED', 'LANDING'):
                    return f'{u} is released but not clear of the carousel (z={st.position_enu.z:.2f} m, {st.phase})'
        return None

    # ----------------------------------------------------------------- loop
    def tick(self):
        if self.activity == 'INDEXING' and self.measured is not None:
            target, done_idx, finished = self.seq.update(self.measured, self.fresh())
            if done_idx is not None:
                self.last_index = done_idx
                self.last_progress = self.get_clock().now()
                self.get_logger().info(f'measured waypoint {done_idx} ({self.w[done_idx].name}) reached')
            if target is not None:
                self.cmd = list(target)
            if finished:
                self.activity = 'IDLE'
                self.paused_goal = None
                station, _ = self.station()
                self.log.emit(self.active_req or '-', 'INDEX', 'COMPLETED', f'measured station {station}')
                self.active_req = None
            elif (self.get_clock().now() - self.last_progress).nanoseconds * 1e-9 > self.get_parameter('stall_s').value:
                self.activity = 'FAULT'
                self.paused_goal = self.goal      # the same index command resumes after the cause is cleared
                self.seq.stop()
                self.cmd = list(self.measured)
                self.log.emit(self.active_req or '-', 'INDEX', 'FAILED', 'no measured progress (stall)')
                self.active_req = None
        self.publish_state()

    def republish(self):
        for j, v in zip(JOINTS, self.cmd):
            self.cmd_pubs[j].publish(Float64(data=float(v)))

    def publish_state(self):
        m = CarouselState()
        m.stamp = self.get_clock().now().to_msg()
        m.joint_names = list(JOINTS)
        m.measured_position_m = [float(v) for v in (self.measured or [])]
        m.commanded_position_m = [float(v) for v in self.cmd]
        m.measured_fresh = self.fresh()
        m.moving = self.activity == 'INDEXING'
        station, index = self.station()
        m.station = station
        m.waypoint_index = index if index >= 0 else (self.last_index if self.measured and reached(self.measured, self.w[self.last_index].q, self.tol) else -1)
        m.target_index = self.seq.current if self.seq.current is not None else -1
        m.accessible_docks = list(self.w[index].accessible) if index >= 0 and self.activity != 'INDEXING' else []
        m.latched = [u for u in UAVS if self.latch.get(u) == 'attached']
        m.activity = self.activity
        self.state_pub.publish(m)


def main():
    rclpy.init()
    rclpy.spin(CarouselSupervisor())


if __name__ == '__main__':
    main()
