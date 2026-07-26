from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    sensors = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('farm_sensor_bringup'), 'launch', 'sensors.launch.py'])),
        launch_arguments={
            'start_velodyne': LaunchConfiguration('start_velodyne'),
            'start_imu': LaunchConfiguration('start_imu'),
            'start_realsense': LaunchConfiguration('start_realsense'),
            'publish_static_tf': LaunchConfiguration('publish_static_tf'),
            'camera_extrinsics_configured': LaunchConfiguration('camera_extrinsics_configured'),
        }.items(),
        condition=UnlessCondition(LaunchConfiguration('ui_only')))
    core = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('collection_bringup'), 'launch', 'collection_core.launch.py'])))
    ui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('collection_bringup'), 'launch', 'collection_ui.launch.py'])),
        launch_arguments={
            'start_rqt': LaunchConfiguration('start_rqt'),
            'start_rviz': LaunchConfiguration('start_rviz'),
            'start_image_view': LaunchConfiguration('start_image_view'),
        }.items())
    glim = Node(
        package='glim_ros',
        executable='glim_rosnode',
        name='glim_ros',
        parameters=[{
            'config_path': '/workspace/farm_ws/config/vlp16_fdilink',
            'dump_path': '/workspace/farm_ws/mapping_sessions/glim_live_dump',
        }],
        output='screen',
        condition=IfCondition(LaunchConfiguration('start_glim')))
    return LaunchDescription([
        DeclareLaunchArgument('start_velodyne', default_value='true'),
        DeclareLaunchArgument('start_imu', default_value='true'),
        DeclareLaunchArgument('start_realsense', default_value='true'),
        DeclareLaunchArgument('publish_static_tf', default_value='true'),
        DeclareLaunchArgument('camera_extrinsics_configured', default_value='false'),
        DeclareLaunchArgument('start_glim', default_value='false'),
        DeclareLaunchArgument('start_rqt', default_value='true'),
        DeclareLaunchArgument('start_rviz', default_value='true'),
        DeclareLaunchArgument('start_image_view', default_value='true'),
        DeclareLaunchArgument('ui_only', default_value='false'),
        sensors, core, ui, glim,
        LogInfo(
            msg='GLIM remains disabled until LiDAR and IMU message validation passes.',
            condition=UnlessCondition(LaunchConfiguration('start_glim'))),
    ])
