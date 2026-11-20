import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    cfg = os.path.join(get_package_share_directory('tins8_bringup'), 'rviz', 'tins8.rviz')
    return LaunchDescription([Node(package='rviz2', executable='rviz2', arguments=['-d', cfg], output='screen',
                                   parameters=[{'use_sim_time': True}])])
