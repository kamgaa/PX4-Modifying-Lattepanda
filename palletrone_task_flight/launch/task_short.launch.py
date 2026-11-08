from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = Path(get_package_share_directory('palletrone_task_flight')) / 'config' / 'task_short_live.yaml'
    return LaunchDescription([
        DeclareLaunchArgument('config_file', default_value=str(config)),
        Node(package='palletrone_task_flight', executable='task_flight.py',
             name='task_flight', parameters=[LaunchConfiguration('config_file')],
             output='screen', respawn=False),
    ])
