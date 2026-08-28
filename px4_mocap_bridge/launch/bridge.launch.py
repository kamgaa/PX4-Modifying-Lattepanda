from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import os


def generate_launch_description():
    package_share = get_package_share_directory('px4_mocap_bridge')
    default_config = os.path.join(package_share, 'config', 'bridge.yaml')

    config_file = LaunchConfiguration('config_file')
    rigid_body_name = LaunchConfiguration('rigid_body_name')
    input_topic = LaunchConfiguration('input_topic')
    output_topic = LaunchConfiguration('output_topic')

    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file',
            default_value=default_config,
            description='Path to the px4_mocap_bridge ROS parameter YAML file.',
        ),
        DeclareLaunchArgument(
            'rigid_body_name',
            default_value='drone',
            description='NamedPose rigid body to forward to PX4.',
        ),
        DeclareLaunchArgument(
            'input_topic',
            default_value='/poses',
            description='NamedPoseArrayV2 input topic.',
        ),
        DeclareLaunchArgument(
            'output_topic',
            default_value='/fmu/in/vehicle_visual_odometry',
            description='PX4 VehicleOdometry input topic.',
        ),
        Node(
            package='px4_mocap_bridge',
            executable='mocap_bridge_node',
            name='px4_mocap_bridge',
            output='screen',
            parameters=[
                config_file,
                {
                    'rigid_body_name': rigid_body_name,
                    'input_topic': input_topic,
                    'output_topic': output_topic,
                },
            ],
        ),
    ])
