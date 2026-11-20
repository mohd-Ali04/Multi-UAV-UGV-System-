"""Per-aircraft PX4 offboard agent (one instance per active UAV, e.g. uav01, uav02).

Each agent talks only to its own PX4 namespace, which is what makes per-aircraft command
isolation testable: a command to uav01 is published on /uav01/fmu/in/* only.

Interfaces (shown for uav01)
  sub /uav01/fmu/out/vehicle_local_position, vehicle_status, vehicle_attitude  (px4_msgs)
  sub /ugv/latch/uav01/state       std_msgs/String  (must be 'detached' before take-off)
  sub /uav01/control_owner         std_msgs/String  ('pursuit' hands setpoint authority to pursuit_node)
  pub /uav01/fmu/in/offboard_control_mode, trajectory_setpoint, vehicle_command (>= 20 Hz while owner)
  pub /uav01/status                tins8_interfaces/UavStatus
  srv /uav01/command               tins8_interfaces/UavCommand
"""
import math

import numpy as np
import rclpy
from px4_msgs.msg import (OffboardControlMode, TrajectorySetpoint, VehicleAttitude, VehicleCommand,
                          VehicleLocalPosition, VehicleStatus)
from rclpy.node import Node
from std_msgs.msg import String

from tins8_interfaces.msg import UavStatus
from tins8_interfaces.srv import UavCommand

from .common import CommandLog, LATCHED_QOS, PX4_OUT_QOS, px4_topic, named_point, new_request_id, scenario
from .geo import LocalTangentPlane, enu_to_ned, heading_ned_to_yaw_enu, ned_to_enu, yaw_enu_to_heading_ned

NAV_STATE = {0: 'MANUAL', 1: 'ALTCTL', 2: 'POSCTL', 3: 'AUTO_MISSION', 4: 'AUTO_LOITER', 5: 'AUTO_RTL',
             10: 'ACRO', 12: 'DESCEND', 13: 'TERMINATION', 14: 'OFFBOARD', 15: 'STAB', 17: 'AUTO_TAKEOFF',
             18: 'AUTO_LAND', 19: 'AUTO_FOLLOW_TARGET', 20: 'AUTO_PRECLAND', 21: 'ORBIT'}
AIRBORNE_PHASES = ('TAKEOFF', 'TRANSIT', 'HOLD', 'PURSUIT')


class UavAgent(Node):
    def __init__(self):
        super().__init__('uav_agent')
        self.declare_parameter('uav_id', 'uav01')
        self.declare_parameter('px4_namespace', '')
        self.declare_parameter('target_system', 1)
        self.declare_parameter('require_latch_release', True)
        self.declare_parameter('cruise_speed', 3.0)
        self.declare_parameter('climb_speed', 1.5)
        self.declare_parameter('takeoff_clearance_m', 3.0)
        self.declare_parameter('arrive_tolerance_m', 0.5)
        self.declare_parameter('arrive_dwell_s', 1.5)
        self.declare_parameter('fresh_s', 0.5)
        self.uav = self.get_parameter('uav_id').value
        ns = self.get_parameter('px4_namespace').value or f'/{self.uav}'
        self.sysid = int(self.get_parameter('target_system').value)
        cfg = scenario()
        self.cfg = cfg
        o = cfg['world_origin']
        self.ltp = LocalTangentPlane(o['lat'], o['lon'], o['alt'])
        self.log = CommandLog(self, self.uav)

        self.lpos = None
        self.lpos_t = None
        self.status = None
        self.latch = 'unknown'
        self.owner = 'agent'
        self.phase = 'DOCKED'
        self.goal_enu = None
        self.goal_yaw = None
        self.goal_name = ''
        self.carrot_enu = None
        self.climb_done = False
        self.arrive_since = None
        self.req = None             # (request_id, command, argument)
        self.last_cmd = ('', '')
        self.stream_count = 0
        self.last_mode_cmd_t = None

        self.pub_mode = self.create_publisher(OffboardControlMode, px4_topic(ns, 'in', 'offboard_control_mode', OffboardControlMode), 10)
        self.pub_sp = self.create_publisher(TrajectorySetpoint, px4_topic(ns, 'in', 'trajectory_setpoint', TrajectorySetpoint), 10)
        self.pub_cmd = self.create_publisher(VehicleCommand, px4_topic(ns, 'in', 'vehicle_command', VehicleCommand), 10)
        self.create_subscription(VehicleLocalPosition, px4_topic(ns, 'out', 'vehicle_local_position', VehicleLocalPosition), self.on_lpos, PX4_OUT_QOS)
        self.create_subscription(VehicleStatus, px4_topic(ns, 'out', 'vehicle_status', VehicleStatus), self.on_status, PX4_OUT_QOS)
        self.create_subscription(String, f'/ugv/latch/{self.uav}/state', self.on_latch, LATCHED_QOS)
        self.land_after_arrival = False
        self.create_subscription(String, f'/{self.uav}/control_owner', self.on_owner, LATCHED_QOS)
        self.pub_status = self.create_publisher(UavStatus, f'/{self.uav}/status', 10)
        self.create_service(UavCommand, f'/{self.uav}/command', self.on_command)
        self.create_timer(1.0 / 30.0, self.tick)
        self.create_timer(0.2, self.publish_status)

    # ---------------------------------------------------------------- telemetry
    def now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_lpos(self, m):
        self.lpos, self.lpos_t = m, self.now_s()

    def on_status(self, m):
        self.status = m

    def on_latch(self, m):
        self.latch = m.data

    def on_owner(self, m):
        if m.data == self.owner:
            return
        self.owner = m.data
        if m.data == 'pursuit':
            self.phase = 'PURSUIT'
            self._finish('CANCELED', 'setpoint authority handed to pursuit_node')
        elif m.data == 'agent':
            pos = self.position_enu()
            if pos is not None:
                self._set_goal(pos, self.yaw_enu(), 'pursuit_exit_hold')
                self.carrot_enu = pos.copy()
                self.climb_done = True
                self.phase = 'HOLD'
            self.get_logger().info('setpoint authority returned by pursuit_node; holding position')

    def fresh(self):
        return self.lpos_t is not None and self.now_s() - self.lpos_t <= self.get_parameter('fresh_s').value

    def ref_enu(self):
        m = self.lpos
        return self.ltp.to_enu(m.ref_lat, m.ref_lon, m.ref_alt)

    def position_enu(self):
        if self.lpos is None or not self.lpos.xy_global:
            return None
        return self.ref_enu() + ned_to_enu([self.lpos.x, self.lpos.y, self.lpos.z])

    def yaw_enu(self):
        return heading_ned_to_yaw_enu(self.lpos.heading) if self.lpos is not None else 0.0

    def armed(self):
        return self.status is not None and self.status.arming_state == VehicleStatus.ARMING_STATE_ARMED

    def nav_state(self):
        return NAV_STATE.get(self.status.nav_state, str(self.status.nav_state)) if self.status else 'UNKNOWN'

    # ---------------------------------------------------------------- commands
    def on_command(self, req, res):
        cmd = req.command.strip().upper()
        arg = req.argument.strip()
        res.request_id = new_request_id(self.uav)
        self.log.emit(res.request_id, cmd, 'REQUESTED', argument=arg)
        if req.uav_id and req.uav_id != self.uav:
            res.accepted, res.message = False, f'addressed to {req.uav_id}, this agent is {self.uav}'
        else:
            res.accepted, res.message = self._dispatch(cmd, arg)
        if res.accepted:
            self._finish('CANCELED', f'superseded by {cmd} {arg}')
            self.req = (res.request_id, cmd, arg)
            self.last_cmd = (f'{cmd} {arg}'.strip(), 'ACCEPTED')
            self.log.emit(res.request_id, cmd, 'ACCEPTED', res.message, arg)
            if cmd == 'HOLD':
                self._finish('COMPLETED', 'holding current position')
        else:
            self.last_cmd = (f'{cmd} {arg}'.strip(), 'REJECTED')
            self.log.emit(res.request_id, cmd, 'REJECTED', res.message, arg)
        return res

    def _dispatch(self, cmd, arg):
        if not self.fresh():
            return False, 'no fresh PX4 telemetry'
        if self.owner == 'pursuit' and cmd != 'LAND':
            return False, 'pursuit_node owns the setpoints; stop pursuit first'
        pos = self.position_enu()
        if pos is None:
            return False, 'PX4 has no global position reference yet'
        docked = self.phase in ('DOCKED', 'LANDED')
        if docked and cmd in ('TAKEOFF_HOLD', 'GOTO_OBSERVATION', 'GOTO_WAYPOINT'):
            if self.get_parameter('require_latch_release').value and self.latch != 'detached':
                return False, f'dock latch is {self.latch}; release it with the carousel first'
        if cmd == 'TAKEOFF_HOLD':
            if not docked:
                return False, f'already {self.phase}'
            goal = pos + np.array([0.0, 0.0, self.get_parameter('takeoff_clearance_m').value])
            self._begin_flight(goal, self.yaw_enu(), 'takeoff_hold', pos)
            return True, f'take off and hold at z={goal[2]:.1f} m'
        if cmd in ('GOTO_OBSERVATION', 'GOTO_WAYPOINT'):
            key = 'observation_points' if cmd == 'GOTO_OBSERVATION' else 'inspection_waypoints'
            p = self.cfg.get(key, {}).get(arg)
            if p is None:
                return False, f'unknown {key[:-1]} {arg!r}; choose from {sorted(self.cfg.get(key, {}))}'
            goal = np.array([p['x'], p['y'], p['z']], dtype=float)
            yaw = math.radians(p.get('yaw_deg', 0.0))
            if docked:
                self._begin_flight(goal, yaw, arg, pos)
            else:
                self._set_goal(goal, yaw, arg)
                self.phase = 'TRANSIT'
            return True, f'flying to {arg} ({goal[0]:.1f}, {goal[1]:.1f}, {goal[2]:.1f})'
        if cmd == 'HOLD':
            if self.phase not in AIRBORNE_PHASES:
                return False, f'not airborne ({self.phase})'
            self._set_goal(pos, self.yaw_enu(), 'hold_here')
            self.carrot_enu = pos.copy()
            self.phase = 'HOLD'
            return True, 'holding current position'
        if cmd == 'LAND':
            # Three landing cases, kept distinct on purpose:
            #   ground  ('', 'here' or a landing pad): implemented, PX4 AUTO_LAND on open ground
            #   ugv     : NOT implemented (moving deck, precision landing, re-latching)
            #   restow  : NOT implemented (dock alignment, re-latch, reverse guarded exchange)
            if arg == 'ugv':
                return False, ('landing on the UGV is not implemented in this build: it needs precision '
                               'landing on the deck and re-latching the dock; land on a ground pad instead')
            if arg in ('restow', 'restowage'):
                return False, ('re-stowage into the carousel is not implemented in this build: it needs dock '
                               'alignment, re-latching and the reverse guarded exchange')
            if self.phase not in AIRBORNE_PHASES:
                return False, f'not airborne ({self.phase})'
            if arg in ('', 'here', 'ground'):
                self.phase = 'LANDING'
                self.land_after_arrival = False
                self._vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND, p4=math.nan, p5=math.nan, p6=math.nan, p7=math.nan)
                return True, 'PX4 AUTO_LAND on the ground below (ground case; not the UGV)'
            pad = self.cfg.get('landing_pads', {}).get(arg)
            if pad is None:
                return False, f"unknown landing target {arg!r}; use here, {', '.join(sorted(self.cfg.get('landing_pads', {})))}, ugv or restow"
            self._set_goal(np.array([pad['x'], pad['y'], pos[2]], float), self.yaw_enu(), arg)
            self.phase = 'TRANSIT'
            self.land_after_arrival = True
            return True, f'flying to ground pad {arg} ({pad["x"]:.1f}, {pad["y"]:.1f}), then PX4 AUTO_LAND'
        return False, f'unknown command {cmd}'

    def _set_goal(self, goal, yaw, name):
        self.goal_enu, self.goal_yaw, self.goal_name = np.asarray(goal, float), yaw, name
        self.arrive_since = None

    def _begin_flight(self, goal, yaw, name, start):
        self._set_goal(goal, yaw, name)
        self.carrot_enu = start.copy()
        self.climb_target_z = start[2] + self.get_parameter('takeoff_clearance_m').value
        self.climb_done = False
        self.stream_count = 0
        self.phase = 'TAKEOFF'
        self.start_yaw = self.yaw_enu()

    def _finish(self, state, detail):
        if self.req is None:
            return
        rid, cmd, arg = self.req
        self.log.emit(rid, cmd, state, detail, arg)
        self.last_cmd = (f'{cmd} {arg}'.strip(), state)
        self.req = None

    def _vehicle_command(self, command, p1=0.0, p2=0.0, p4=0.0, p5=0.0, p6=0.0, p7=0.0):
        m = VehicleCommand()
        m.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        m.command = command
        m.param1, m.param2, m.param4, m.param5, m.param6, m.param7 = float(p1), float(p2), float(p4), float(p5), float(p6), float(p7)
        m.target_system, m.target_component = self.sysid, 1
        m.source_system, m.source_component = 1, 1
        m.from_external = True
        self.pub_cmd.publish(m)

    # ---------------------------------------------------------------- control loop
    def tick(self):
        if self.status is not None and self.phase == 'LANDING' and not self.armed():
            self.phase = 'LANDED'
            self._finish('COMPLETED', 'disarmed on the ground after AUTO_LAND')
        if self.owner != 'agent' or self.phase not in ('TAKEOFF', 'TRANSIT', 'HOLD') or self.goal_enu is None:
            return
        if not self.fresh():
            return              # never command from stale telemetry; PX4 failsafe covers loss
        dt = 1.0 / 30.0
        pos = self.position_enu()
        if self.phase == 'TAKEOFF' and not self.climb_done:
            # Vertical climb first so the aircraft clears the carousel before moving sideways.
            step = self.get_parameter('climb_speed').value * dt
            self.carrot_enu[2] = min(self.climb_target_z, self.carrot_enu[2] + step) if self.armed() else pos[2]
            if pos[2] >= self.climb_target_z - 0.4:
                self.climb_done = True
                self.phase = 'TRANSIT'
            yaw = self.start_yaw
        else:
            delta = self.goal_enu - self.carrot_enu
            dist = float(np.linalg.norm(delta))
            step = self.get_parameter('cruise_speed').value * dt
            self.carrot_enu = self.goal_enu.copy() if dist <= step else self.carrot_enu + delta / dist * step
            yaw = self.goal_yaw
            err = float(np.linalg.norm(self.goal_enu - pos))
            speed = math.hypot(self.lpos.vx, self.lpos.vy)
            if self.phase == 'TRANSIT':
                if err < self.get_parameter('arrive_tolerance_m').value and speed < 0.5:
                    self.arrive_since = self.arrive_since or self.now_s()
                    if self.now_s() - self.arrive_since >= self.get_parameter('arrive_dwell_s').value:
                        if self.land_after_arrival:
                            self.land_after_arrival = False
                            self.phase = 'LANDING'
                            self.get_logger().info(f'over {self.goal_name} (error {err:.2f} m): AUTO_LAND')
                            self._vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND, p4=math.nan, p5=math.nan, p6=math.nan, p7=math.nan)
                            return
                        self.phase = 'HOLD'
                        self._finish('COMPLETED', f'arrived at {self.goal_name}, error {err:.2f} m; holding')
                else:
                    self.arrive_since = None
        self._stream(self.carrot_enu, yaw)
        if self.phase == 'TAKEOFF' and self.stream_count > 15:
            t = self.now_s()
            if not (self.armed() and self.nav_state() == 'OFFBOARD') and (self.last_mode_cmd_t is None or t - self.last_mode_cmd_t > 1.0):
                self._vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
                self._vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
                self.last_mode_cmd_t = t

    def _stream(self, target_enu, yaw_enu):
        now_us = int(self.get_clock().now().nanoseconds / 1000)
        mode = OffboardControlMode()
        mode.timestamp = now_us
        mode.position = True
        self.pub_mode.publish(mode)
        sp = TrajectorySetpoint()
        sp.timestamp = now_us
        ned = enu_to_ned(np.asarray(target_enu) - self.ref_enu())
        sp.position = [float(v) for v in ned]
        sp.velocity = [math.nan] * 3
        sp.acceleration = [math.nan] * 3
        sp.yaw = float(yaw_enu_to_heading_ned(yaw_enu))
        sp.yawspeed = math.nan
        self.pub_sp.publish(sp)
        self.stream_count += 1

    def publish_status(self):
        m = UavStatus()
        m.stamp = self.get_clock().now().to_msg()
        m.uav_id = self.uav
        m.connected = self.fresh()
        m.telemetry_age_s = float(self.now_s() - self.lpos_t) if self.lpos_t else -1.0
        m.armed = self.armed()
        m.nav_state = self.nav_state()
        m.phase = self.phase
        m.control_owner = self.owner
        pos = self.position_enu() if self.lpos is not None else None
        if pos is not None:
            m.position_enu.x, m.position_enu.y, m.position_enu.z = (float(v) for v in pos)
            m.yaw_enu_rad = float(self.yaw_enu())
            m.speed_mps = float(math.sqrt(self.lpos.vx ** 2 + self.lpos.vy ** 2 + self.lpos.vz ** 2))
        if self.goal_enu is not None:
            m.hold_target_enu.x, m.hold_target_enu.y, m.hold_target_enu.z = (float(v) for v in self.goal_enu)
            if pos is not None:
                m.hold_error_m = float(np.linalg.norm(self.goal_enu - pos))
        m.target_name = self.goal_name
        m.last_command, m.last_command_state = self.last_cmd
        self.pub_status.publish(m)


def main():
    rclpy.init()
    rclpy.spin(UavAgent())


if __name__ == '__main__':
    main()
