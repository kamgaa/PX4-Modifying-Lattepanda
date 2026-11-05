import os

from ament_index_python.packages import (
    PackageNotFoundError,
    get_package_share_directory,
)
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _require_motion_capture_tracking():
    try:
        get_package_share_directory('motion_capture_tracking')
    except PackageNotFoundError as exc:
        raise RuntimeError(
            "Required ROS 2 package 'motion_capture_tracking' was not found. "
            'Clone the ros2 branch of '
            'https://github.com/IMRCLab/motion_capture_tracking with '
            '--recurse-submodules, build it in this workspace, and source '
            'install/setup.bash before launching.'
        ) from exc


def generate_launch_description():
    _require_motion_capture_tracking()

    package_share = get_package_share_directory('px4_mocap_bridge')
    bridge_config = os.path.join(package_share, 'config', 'bridge.yaml')
    optitrack_config = os.path.join(package_share, 'config', 'optitrack.yaml')

    hostname = LaunchConfiguration('hostname')
    rigid_body_name = LaunchConfiguration('rigid_body_name')

    return LaunchDescription([
        DeclareLaunchArgument(
            'hostname',
            default_value='192.168.10.115',
            description='Hostname or IP address of the Motive computer.',
        ),
        DeclareLaunchArgument(
            'rigid_body_name',
            default_value='drone',
            description='Motive rigid body name to forward to PX4.',
        ),
        Node(
            package='motion_capture_tracking',
            executable='motion_capture_tracking_node',
            name='motion_capture_tracking',
            output='screen',
            parameters=[optitrack_config, {'hostname': hostname}],
        ),
        Node(
            package='px4_mocap_bridge',
            executable='mocap_bridge_node',
            name='px4_mocap_bridge',
            output='screen',
            parameters=[
                bridge_config,
                {'rigid_body_name': rigid_body_name},
            ],
        ),
    ])
