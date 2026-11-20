"""Nodes for one active aircraft: PX4 agent, and for camera aircraft YOLOX + tracker + pursuit."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    uav = LaunchConfiguration('uav_id')
    sim = {'use_sim_time': True}
    return LaunchDescription([
        DeclareLaunchArgument('uav_id', default_value='uav01'),
        DeclareLaunchArgument('target_system', default_value='1'),
        DeclareLaunchArgument('require_latch_release', default_value='true'),
        DeclareLaunchArgument('perception', default_value='true'),
        Node(package='tins8_mission', executable='uav_agent', name=['uav_agent_', uav], output='screen',
             parameters=[sim, {'uav_id': uav, 'target_system': LaunchConfiguration('target_system'),
                               'require_latch_release': LaunchConfiguration('require_latch_release')}]),
        Node(package='tins8_mission', executable='yolox_detector', name=['yolox_detector_', uav], output='screen',
             parameters=[sim, {'uav_id': uav}], condition=IfCondition(LaunchConfiguration('perception'))),
        Node(package='tins8_mission', executable='target_tracker', name=['target_tracker_', uav], output='screen',
             parameters=[sim, {'uav_id': uav}], condition=IfCondition(LaunchConfiguration('perception'))),
        Node(package='tins8_mission', executable='pursuit_node', name=['pursuit_node_', uav], output='screen',
             parameters=[sim, {'uav_id': uav}], condition=IfCondition(LaunchConfiguration('perception'))),
    ])
