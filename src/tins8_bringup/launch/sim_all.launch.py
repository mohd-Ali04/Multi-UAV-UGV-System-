"""One terminal entry point for the full simulation:
Gazebo + bridge, Micro XRCE-DDS Agent, PX4 SITL for uav01 and uav02, all nodes, RViz and the GUI.

ros2 launch tins8_bringup sim_all.launch.py [rviz:=true] [gui:=true] [gz_gui:=true]
(The tabbed launcher scripts/run_tabs.sh starts the same pieces in separate named tabs.)
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    share = get_package_share_directory('tins8_bringup')
    ws = os.environ.get('TINS8_WS', '')
    scripts = os.path.join(ws, 'scripts')
    inc = lambda f, **a: IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(share, 'launch', f)),
                                                  launch_arguments=a.items())
    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('gz_gui', default_value='true'),
        DeclareLaunchArgument('run_name', default_value=''),
        inc('gazebo.launch.py', world='fieldlab', gui=LaunchConfiguration('gz_gui')),
        ExecuteProcess(cmd=['bash', os.path.join(scripts, 'start_agent.sh')], name='xrce_agent', output='log'),
        # PX4 waits for the Gazebo world itself; the short delays only keep the logs readable.
        TimerAction(period=4.0, actions=[ExecuteProcess(cmd=['bash', os.path.join(scripts, 'start_px4.sh'), 'uav01', 'fieldlab', '--daemon'], name='px4_uav01', output='log')]),
        TimerAction(period=6.0, actions=[ExecuteProcess(cmd=['bash', os.path.join(scripts, 'start_px4.sh'), 'uav02', 'fieldlab', '--daemon'], name='px4_uav02', output='log')]),
        inc('nodes.launch.py', world='fieldlab', run_name=LaunchConfiguration('run_name')),
        TimerAction(period=8.0, actions=[inc('rviz.launch.py')], condition=IfCondition(LaunchConfiguration('rviz'))),
        TimerAction(period=8.0, actions=[ExecuteProcess(cmd=['bash', os.path.join(scripts, 'start_gui.sh')], name='tins8_gui', output='screen')],
                    condition=IfCondition(LaunchConfiguration('gui'))),
    ])
