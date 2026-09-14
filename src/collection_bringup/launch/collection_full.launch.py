"""Launch the confirmed UI-only collection and mapping preview stack."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Create the collection core and GUI launch description."""
    core = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('collection_bringup'),
            'launch',
            'collection_core.launch.py',
        ])),
        launch_arguments={
            'start_coverage': LaunchConfiguration('start_coverage'),
            'mapping_odom_topic': LaunchConfiguration('mapping_odom_topic'),
            'recording_enabled': LaunchConfiguration('recording_enabled'),
            'session_root': LaunchConfiguration('session_root'),
            'health_config_file': LaunchConfiguration('health_config_file'),
            'record_topics': LaunchConfiguration('record_topics'),
            'storage_id': LaunchConfiguration('storage_id'),
            'compression_mode': LaunchConfiguration('compression_mode'),
            'compression_format': LaunchConfiguration('compression_format'),
        }.items(),
    )
    ui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('collection_bringup'),
            'launch',
            'collection_ui.launch.py',
        ])),
        launch_arguments={
            'start_rqt': LaunchConfiguration('start_rqt'),
            'start_rviz': LaunchConfiguration('start_rviz'),
            'start_image_view': LaunchConfiguration('start_image_view'),
            'rviz_config': LaunchConfiguration('rviz_config'),
            'ui_only_default': LaunchConfiguration('ui_only_default'),
        }.items(),
    )
    return LaunchDescription([
        DeclareLaunchArgument('ui_only', default_value='true'),
        DeclareLaunchArgument('start_rqt', default_value='true'),
        DeclareLaunchArgument('start_rviz', default_value='true'),
        DeclareLaunchArgument('start_image_view', default_value='false'),
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
        DeclareLaunchArgument('ui_only_default', default_value='true'),
        DeclareLaunchArgument(
            'rviz_config',
            default_value='/workspace/farm_ws/rviz/collection.rviz',
        ),
        DeclareLaunchArgument('start_glim', default_value='false'),
        DeclareLaunchArgument('start_drivers', default_value='false'),
        core,
        ui,
        LogInfo(
            msg=(
                'GLIM mapping requested; this collection launch does not own '
                'the upstream mapping process.'
            ),
            condition=IfCondition(LaunchConfiguration('start_glim')),
        ),
        LogInfo(
            msg='Drivers requested but no hardware driver launch is configured.',
            condition=IfCondition(LaunchConfiguration('start_drivers')),
        ),
    ])
