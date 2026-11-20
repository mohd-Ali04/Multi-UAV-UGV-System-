"""RViz display only; independent joint controls do not enforce the guarded path."""
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution


def generate_launch_description():
    share = get_package_share_directory("tins_description")
    aircraft_mode = LaunchConfiguration("aircraft_mode")
    description = ParameterValue(Command([
        FindExecutable(name="xacro"), " ",
        PathJoinSubstitution([share, "urdf", "tins_ugv_carousel.urdf.xacro"]),
        " aircraft_mode:=", aircraft_mode,
    ]), value_type=str)
    return LaunchDescription([
        DeclareLaunchArgument("aircraft_mode", default_value="static_visual",
                              choices=["none", "static_visual"]),
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             parameters=[{"robot_description": description}]),
        Node(package="joint_state_publisher_gui", executable="joint_state_publisher_gui"),
        Node(package="rviz2", executable="rviz2"),
    ])
