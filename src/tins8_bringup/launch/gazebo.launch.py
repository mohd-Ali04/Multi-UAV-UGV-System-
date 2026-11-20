"""Gazebo Harmonic (server + optional GUI) and the ros_gz bridge.

ros2 launch tins8_bringup gazebo.launch.py world:=fieldlab gui:=true
"""
import os
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import TimerAction, DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def setup(context):
    world = LaunchConfiguration('world').perform(context)
    gz_share = get_package_share_directory('tins8_gazebo')
    desc_parent = os.path.dirname(get_package_share_directory('tins_description'))
    world_file = os.path.join(gz_share, 'worlds', 'fieldlab.sdf' if world == 'fieldlab' else 'minimal_px4.sdf')
    world_name = 'fieldlab' if world == 'fieldlab' else 'minimal'
    resource = os.pathsep.join([os.path.join(gz_share, 'models'), desc_parent, os.environ.get('GZ_SIM_RESOURCE_PATH', '')])
    # Same bridge map for both worlds; only the world name in the pose topic differs.
    cfg = open(os.path.join(get_package_share_directory('tins8_bringup'), 'config', 'bridge.yaml')).read()
    cfg = cfg.replace('/world/fieldlab/', f'/world/{world_name}/')
    bridge_file = os.path.join(tempfile.gettempdir(), f'tins8_bridge_{world_name}_{os.getuid()}.yaml')
    open(bridge_file, 'w').write(cfg)
    server = ['gz', 'sim', '-s', '-r', '-v', '2', world_file]
    if LaunchConfiguration('headless_rendering').perform(context) == 'true':
        server.insert(3, '--headless-rendering')
    return [
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', resource),
        # gz_relay first: it must be subscribed when the dock latches report "attached" at the
        # first physics step (DetachableJoint publishes its state only on change).
        Node(package='tins8_mission', executable='gz_relay', output='screen',
             parameters=[{'world': world_name, 'use_sim_time': False}]),
        TimerAction(period=2.0, actions=[ExecuteProcess(cmd=server, output='screen', name='gz_server')]),
        ExecuteProcess(cmd=['gz', 'sim', '-g', '-v', '1', '--gui-config', os.path.join(gz_share, 'config', 'gui.config')],
                       output='screen', name='gz_gui',
                       condition=IfCondition(LaunchConfiguration('gui'))),
        Node(package='ros_gz_bridge', executable='parameter_bridge', name='gz_bridge', output='screen',
             parameters=[{'config_file': bridge_file, 'use_sim_time': True}]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='fieldlab', description='fieldlab | minimal'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('headless_rendering', default_value='false',
                              description='true for machines without a display (cameras still render)'),
        OpaqueFunction(function=setup),
    ])
