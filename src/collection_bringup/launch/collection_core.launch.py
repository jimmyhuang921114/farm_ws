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
        Node(
            package='collection_manager',
            executable='collection_manager',
            parameters=[{
                'session_root': '/workspace/farm_ws/mapping_sessions',
            }],
        ),
        Node(
            package='sensor_health_monitor',
            executable='sensor_health_monitor',
            parameters=['/workspace/farm_ws/config/sensors/health.yaml'],
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
