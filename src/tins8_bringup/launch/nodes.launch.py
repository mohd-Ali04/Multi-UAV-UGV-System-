"""All project ROS 2 nodes.  Start after Gazebo (and before or after PX4: agents wait for telemetry).

ros2 launch tins8_bringup nodes.launch.py [world:=fieldlab] [run_name:=demo1]
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    sim = {'use_sim_time': True}
    base = int(os.environ.get('TINS8_PX4_INSTANCE_BASE', '0'))
    urdf = open(os.path.join(get_package_share_directory('tins_description'), 'urdf', 'tins_ugv_carousel.urdf')).read()
    uav_launch = os.path.join(get_package_share_directory('tins8_bringup'), 'launch', 'uav.launch.py')
    fieldlab = IfCondition(PythonExpression(["'", LaunchConfiguration('world'), "' == 'fieldlab'"]))
    minimal = IfCondition(PythonExpression(["'", LaunchConfiguration('world'), "' == 'minimal'"]))

    def uav(uid, sysid, cond, latch='true', perception='true'):
        return IncludeLaunchDescription(PythonLaunchDescriptionSource(uav_launch), condition=cond,
                                        launch_arguments={'uav_id': uid, 'target_system': str(sysid),
                                                          'require_latch_release': latch,
                                                          'perception': perception}.items())
    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='fieldlab'),
        DeclareLaunchArgument('run_name', default_value=''),
        DeclareLaunchArgument('uav02_perception', default_value='false'),
        uav('uav01', base + 1, fieldlab),
        # uav02 only observes from its point; its YOLOX would halve uav01's detector rate on a
        # laptop CPU.  uav02_perception:=true adds detector, tracker and pursuit for uav02 too.
        uav('uav02', base + 2, fieldlab, perception=LaunchConfiguration('uav02_perception')),
        uav('uav01', base + 1, minimal, latch='false'),
        Node(package='tins8_mission', executable='carousel_supervisor', output='screen', parameters=[sim], condition=fieldlab),
        Node(package='tins8_mission', executable='fleet_manager', output='screen', parameters=[sim], condition=fieldlab),
        Node(package='tins8_mission', executable='mission_server', output='screen', parameters=[sim], condition=fieldlab),
        Node(package='tins8_mission', executable='pedestrian_driver', output='screen', parameters=[sim], condition=fieldlab),
        Node(package='tins8_mission', executable='evaluator', output='screen',
             parameters=[sim, {'run_name': LaunchConfiguration('run_name')}]),
        Node(package='robot_state_publisher', executable='robot_state_publisher', output='screen',
             parameters=[sim, {'robot_description': urdf}], condition=fieldlab),
    ])
