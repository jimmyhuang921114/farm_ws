from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = LaunchConfiguration('velodyne_config_file')
    return LaunchDescription([
        DeclareLaunchArgument(
            'velodyne_config_file',
            default_value='/workspace/farm_ws/config/sensors/velodyne.yaml'),
        Node(
            package='velodyne_driver',
            executable='velodyne_driver_node',
            name='velodyne_driver_node',
            parameters=[config],
            output='screen'),
        Node(
            package='velodyne_pointcloud',
            executable='velodyne_transform_node',
            name='velodyne_transform_node',
            parameters=[config],
            remappings=[('velodyne_packets', '/velodyne_packets'),
                        ('velodyne_points', '/velodyne_points')],
            output='screen'),
    ])
