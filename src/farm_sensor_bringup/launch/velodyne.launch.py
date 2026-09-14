from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml


def launch_nodes(context):
    config_path = Path(LaunchConfiguration('velodyne_config_file').perform(context))
    calibration_path = LaunchConfiguration('velodyne_calibration_file').perform(context)
    data = yaml.safe_load(config_path.read_text(encoding='utf-8')) or {}
    driver_params = data.get('velodyne_driver_node', {}).get('ros__parameters', {})
    transform_params = data.get('velodyne_transform_node', {}).get('ros__parameters', {})
    transform_params['calibration'] = calibration_path
    return [
        Node(
            package='velodyne_driver', executable='velodyne_driver_node',
            name='velodyne_driver_node', parameters=[driver_params], output='screen'),
        Node(
            package='velodyne_pointcloud', executable='velodyne_transform_node',
            name='velodyne_transform_node', parameters=[transform_params],
            remappings=[('velodyne_packets', '/velodyne_packets'),
                        ('velodyne_points', '/velodyne_points')], output='screen'),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'velodyne_config_file',
            default_value='/workspace/farm_ws/config/sensors/velodyne.yaml'),
        DeclareLaunchArgument(
            'velodyne_calibration_file',
            default_value='/workspace/farm_ws/config/velodyne/VLP16db.yaml'),
        OpaqueFunction(function=launch_nodes),
    ])
