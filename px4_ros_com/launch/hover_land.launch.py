"""Only the mission node. Keep communication and actuator nodes independent."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_config = os.path.join(
        get_package_share_directory('px4_ros_com'), 'config', 'hover_land.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('config_file', default_value=default_config),
        Node(
            package='px4_ros_com', executable='palletrone_hover_land.py',
            name='palletrone_hover_land', output='screen', respawn=False,
            parameters=[LaunchConfiguration('config_file')]),
    ])
