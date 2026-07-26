from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'realsense_config_file',
            default_value='/workspace/farm_ws/config/sensors/realsense.yaml'),
        Node(
            package='realsense2_camera',
            executable='realsense2_camera_node',
            namespace='',
            name='camera',
            parameters=[LaunchConfiguration('realsense_config_file')],
            output='screen'),
    ])
