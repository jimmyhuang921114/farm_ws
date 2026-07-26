from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'fdilink_config_file',
            default_value='/workspace/farm_ws/config/sensors/fdilink.yaml'),
        Node(
            package='fdilink_ahrs',
            executable='ahrs_driver_node',
            name='ahrs_bringup',
            parameters=[LaunchConfiguration('fdilink_config_file')],
            output='screen'),
    ])
