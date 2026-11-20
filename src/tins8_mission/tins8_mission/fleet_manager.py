"""Fleet accounting for all eight QAV250 positions, plus RViz support.

The two active aircraft (uav01, uav02) are independent Gazebo/PX4 models; the six stowed
aircraft are massless visuals on their tray links.  This node publishes one consistent
account of all eight and draws each aircraft exactly once in RViz.

  sub /carousel/state, /uav01/status, /uav02/status, /ugv/odom, /ugv/joint_states, /gz/dynamic_poses
  pub /fleet/state      tins8_interfaces/FleetState (2 Hz)
  pub /joint_states     full UGV joint state for robot_state_publisher (trays measured, running gear 0)
  pub /fleet/markers    visualization_msgs/MarkerArray
  tf  world -> odom (UGV start pose), odom -> base_link (UGV wheel odometry)
"""
import math
import os
import xml.etree.ElementTree as E

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf2_msgs.msg import TFMessage
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray

from tins8_interfaces.msg import CarouselState, FleetSlot, FleetState, UavStatus

from .common import ALL_SLOTS, LATCHED_QOS, TRAY_OF, UAVS, scenario

MESH = 'package://tins_description/meshes/aircraft/qav250_visual.stl'


def yaw_quat(yaw):
    return 0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)


class FleetManager(Node):
    def __init__(self):
        super().__init__('fleet_manager')
        self.cfg = scenario()
        urdf = os.path.join(get_package_share_directory('tins_description'), 'urdf', 'tins_ugv_carousel.urdf')
        self.moving_joints = [j.get('name') for j in E.parse(urdf).getroot().findall('joint') if j.get('type') != 'fixed']
        self.carousel = None
        self.status = {}
        self.gt = {}
        self.tray_q = {}
        self.pub_fleet = self.create_publisher(FleetState, '/fleet/state', LATCHED_QOS)
        self.pub_js = self.create_publisher(JointState, '/joint_states', 10)
        self.pub_mk = self.create_publisher(MarkerArray, '/fleet/markers', 10)
        self.tfb = TransformBroadcaster(self)
        self.static = StaticTransformBroadcaster(self)
        u = self.cfg['ugv']
        t = TransformStamped()
        t.header.frame_id, t.child_frame_id = 'world', 'odom'
        t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = float(u['x']), float(u['y']), float(u['z'])
        q = yaw_quat(u['yaw'])
        t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w = q
        self.static.sendTransform(t)
        self.create_subscription(CarouselState, '/carousel/state', lambda m: setattr(self, 'carousel', m), LATCHED_QOS)
        for uav in UAVS:
            self.create_subscription(UavStatus, f'/{uav}/status', lambda m, uav=uav: self.status.__setitem__(uav, m), 10)
        self.create_subscription(Odometry, '/ugv/odom', self.on_odom, 10)
        self.create_subscription(JointState, '/ugv/joint_states', self.on_tray_joints, 10)
        self.create_subscription(TFMessage, '/gz/dynamic_poses', self.on_gt, 10)
        self.create_timer(0.5, self.publish_fleet)
        self.create_timer(0.05, self.publish_joint_states)
        self.create_timer(0.2, self.publish_markers)

    def on_odom(self, m):
        t = TransformStamped()
        t.header.stamp = m.header.stamp
        t.header.frame_id, t.child_frame_id = 'odom', 'base_link'
        p, q = m.pose.pose.position, m.pose.pose.orientation
        t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = p.x, p.y, 0.0
        t.transform.rotation = q
        self.tfb.sendTransform(t)

    def on_tray_joints(self, m):
        self.tray_q = dict(zip(m.name, m.position))

    def on_gt(self, m):
        for t in m.transforms:
            self.gt[t.child_frame_id] = t.transform

    def publish_joint_states(self):
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = list(self.moving_joints)
        js.position = [float(self.tray_q.get(n, 0.0)) for n in js.name]
        self.pub_js.publish(js)

    def slot_state(self, uav):
        if uav not in UAVS:
            return 'STOWED'
        if self.carousel is not None and uav in self.carousel.latched:
            return 'ACTIVE_DOCKED'
        st = self.status.get(uav)
        if st is not None and st.phase in ('TAKEOFF', 'TRANSIT', 'HOLD', 'PURSUIT', 'LANDING'):
            return 'AIRBORNE'
        if st is not None and st.phase == 'LANDED':
            return 'LANDED_GROUND'
        return 'RELEASED'

    def level(self, tray):
        z = self.tray_q.get('tray_a_z_joint' if tray == 'tray_a' else 'tray_b_z_joint', 0.0)
        if tray == 'tray_a':
            return 'upper' if z > -0.0775 else 'lower'
        return 'upper' if z > 0.0775 else 'lower'

    def publish_fleet(self):
        f = FleetState()
        f.stamp = self.get_clock().now().to_msg()
        for uav in ALL_SLOTS:
            s = FleetSlot()
            s.uav_id, s.tray, s.dock_frame = uav, TRAY_OF[uav], f'{uav}_dock'
            s.level = self.level(s.tray)
            s.state = self.slot_state(uav)
            s.independent_model = uav in UAVS
            f.slots.append(s)
        f.stowed = sum(s.state == 'STOWED' for s in f.slots)
        f.active_docked = sum(s.state == 'ACTIVE_DOCKED' for s in f.slots)
        f.airborne = sum(s.state == 'AIRBORNE' for s in f.slots)
        self.pub_fleet.publish(f)

    def _marker(self, ns, mid, frame, mtype, color, scale=(1.0, 1.0, 1.0)):
        m = Marker()
        m.header.frame_id = frame
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns, m.id, m.type, m.action = ns, mid, mtype, Marker.ADD
        m.pose.orientation.w = 1.0
        m.scale.x, m.scale.y, m.scale.z = (float(v) for v in scale)
        m.color.r, m.color.g, m.color.b, m.color.a = (float(v) for v in color)
        return m

    def publish_markers(self):
        arr = MarkerArray()
        for i, uav in enumerate(ALL_SLOTS):
            state = self.slot_state(uav)
            if state in ('STOWED', 'ACTIVE_DOCKED'):
                color = (0.25, 0.27, 0.3, 1.0) if state == 'STOWED' else (0.9, 0.55, 0.1, 1.0)
                m = self._marker('aircraft', i, f'{uav}_dock', Marker.MESH_RESOURCE, color)
                m.mesh_resource = MESH
                arr.markers.append(m)
                frame, z = f'{uav}_dock', 0.12
            else:
                st = self.status.get(uav)
                if st is None:
                    continue
                m = self._marker('aircraft', i, 'world', Marker.MESH_RESOURCE, (0.9, 0.55, 0.1, 1.0), (3.0, 3.0, 3.0))
                m.mesh_resource = MESH
                m.pose.position = st.position_enu
                q = yaw_quat(st.yaw_enu_rad)
                m.pose.orientation.x, m.pose.orientation.y, m.pose.orientation.z, m.pose.orientation.w = q
                arr.markers.append(m)
                frame, z = 'world', st.position_enu.z + 0.6
            t = self._marker('labels', i, frame, Marker.TEXT_VIEW_FACING, (1.0, 1.0, 1.0, 1.0), (0.0, 0.0, 0.08 if frame != 'world' else 0.6))
            t.text = f'{uav.upper()} {state}'
            if frame == 'world':
                st = self.status[uav]
                t.pose.position.x, t.pose.position.y, t.pose.position.z = st.position_enu.x, st.position_enu.y, z
            else:
                t.pose.position.z = z
            arr.markers.append(t)
        k = 100
        for group, color in (('observation_points', (0.3, 0.6, 0.9, 0.6)), ('inspection_waypoints', (0.9, 0.55, 0.2, 0.6))):
            for name, p in self.cfg.get(group, {}).items():
                m = self._marker(group, k, 'world', Marker.SPHERE, color, (0.6, 0.6, 0.6))
                m.pose.position.x, m.pose.position.y, m.pose.position.z = float(p['x']), float(p['y']), float(p['z'])
                arr.markers.append(m)
                t = self._marker(group + '_labels', k, 'world', Marker.TEXT_VIEW_FACING, (1, 1, 1, 1), (0, 0, 0.5))
                t.text = name
                t.pose.position.x, t.pose.position.y, t.pose.position.z = float(p['x']), float(p['y']), float(p['z']) + 0.7
                arr.markers.append(t)
                k += 1
        for name in self.cfg.get('walkers', {}):
            tr = self.gt.get(name)
            if tr is None:
                continue
            m = self._marker('ground_truth_pedestrians', k, 'world', Marker.CYLINDER, (0.8, 0.2, 0.8, 0.5), (0.5, 0.5, 1.75))
            m.pose.position.x, m.pose.position.y, m.pose.position.z = tr.translation.x, tr.translation.y, 0.875
            arr.markers.append(m)
            t = self._marker('ground_truth_pedestrians', k + 50, 'world', Marker.TEXT_VIEW_FACING, (0.9, 0.6, 0.9, 1), (0, 0, 0.5))
            t.text = f'{name} (sim ground truth)'
            t.pose.position.x, t.pose.position.y, t.pose.position.z = tr.translation.x, tr.translation.y, 2.2
            arr.markers.append(t)
            k += 1
        self.pub_mk.publish(arr)


def main():
    rclpy.init()
    rclpy.spin(FleetManager())


if __name__ == '__main__':
    main()
