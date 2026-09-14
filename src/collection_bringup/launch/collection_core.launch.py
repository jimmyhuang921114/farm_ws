"""Launch UI-only collection services, health, and mapping coverage preview."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Create the custom collection core launch description."""
    return LaunchDescription([
        DeclareLaunchArgument('start_coverage', default_value='true'),
        DeclareLaunchArgument('mapping_odom_topic', default_value='/glim_ros/odom'),
        DeclareLaunchArgument('recording_enabled', default_value='false'),
        DeclareLaunchArgument('session_root', default_value='/workspace/farm_ws/mapping_sessions'),
        DeclareLaunchArgument('health_config_file', default_value='/workspace/farm_ws/config/sensors/health.yaml'),
        DeclareLaunchArgument(
            'record_topics',
            default_value=(
                '/livox/lidar /livox/lidar_valid /livox/imu /livox/imu_base '
                '/tf /tf_static /decxin_camera/image_compressed '
                '/decxin_camera/camera_info /glim_ros/odom /glim_ros/points /glim_ros/map'
            ),
        ),
        DeclareLaunchArgument('storage_id', default_value='sqlite3'),
        DeclareLaunchArgument('compression_mode', default_value='file'),
        DeclareLaunchArgument('compression_format', default_value='zstd'),
        Node(
            package='collection_manager',
            executable='collection_manager',
            parameters=[{
                'session_root': LaunchConfiguration('session_root'),
                'recording_enabled': LaunchConfiguration('recording_enabled'),
                'record_topics': LaunchConfiguration('record_topics'),
                'storage_id': LaunchConfiguration('storage_id'),
                'compression_mode': LaunchConfiguration('compression_mode'),
                'compression_format': LaunchConfiguration('compression_format'),
            }],
        ),
        Node(
            package='sensor_health_monitor',
            executable='sensor_health_monitor',
            parameters=[LaunchConfiguration('health_config_file')],
        ),
        Node(
            package='preview_tools',
            executable='coverage_analyzer',
            parameters=[{
                'odom_topic': LaunchConfiguration('mapping_odom_topic'),
            }],
            condition=IfCondition(LaunchConfiguration('start_coverage')),
        ),
    ])
