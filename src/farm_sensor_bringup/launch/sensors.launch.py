from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            LogInfo, OpaqueFunction)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def validate_extrinsics(context):
    semantic = LaunchConfiguration('enable_semantic_projection').perform(context).lower()
    calibrated = LaunchConfiguration('camera_extrinsics_configured').perform(context).lower()
    if semantic in ('true', '1', 'yes', 'on') and calibrated not in ('true', '1', 'yes', 'on'):
        raise RuntimeError(
            'Semantic projection refused: velodyne->camera_link is REQUIRES_CALIBRATION')
    return []


def include(package_launch, condition):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('farm_sensor_bringup'), 'launch', package_launch])),
        condition=IfCondition(LaunchConfiguration(condition)))


def generate_launch_description():
    static_condition = IfCondition(LaunchConfiguration('publish_static_tf'))
    camera_condition = IfCondition(LaunchConfiguration('camera_extrinsics_configured'))
    return LaunchDescription([
        DeclareLaunchArgument('start_velodyne', default_value='true'),
        DeclareLaunchArgument('start_imu', default_value='true'),
        DeclareLaunchArgument('start_realsense', default_value='true'),
        DeclareLaunchArgument('publish_static_tf', default_value='true'),
        DeclareLaunchArgument('camera_extrinsics_configured', default_value='false'),
        DeclareLaunchArgument('enable_semantic_projection', default_value='false'),
        DeclareLaunchArgument('camera_x', default_value='0.0'),
        DeclareLaunchArgument('camera_y', default_value='0.0'),
        DeclareLaunchArgument('camera_z', default_value='0.0'),
        DeclareLaunchArgument('camera_roll', default_value='0.0'),
        DeclareLaunchArgument('camera_pitch', default_value='0.0'),
        DeclareLaunchArgument('camera_yaw', default_value='0.0'),
        OpaqueFunction(function=validate_extrinsics),
        include('velodyne.launch.py', 'start_velodyne'),
        include('fdilink.launch.py', 'start_imu'),
        include('realsense.launch.py', 'start_realsense'),
        Node(
            package='tf2_ros', executable='static_transform_publisher',
            name='base_to_velodyne_static_tf',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'velodyne'],
            condition=static_condition),
        Node(
            package='tf2_ros', executable='static_transform_publisher',
            name='velodyne_to_imu_static_tf',
            arguments=['0', '0', '0.07', '1.5707963267948966', '0',
                       '3.141592653589793', 'velodyne', 'imu_link'],
            condition=static_condition),
        Node(
            package='tf2_ros', executable='static_transform_publisher',
            name='velodyne_to_camera_static_tf',
            arguments=[LaunchConfiguration('camera_x'), LaunchConfiguration('camera_y'),
                       LaunchConfiguration('camera_z'), LaunchConfiguration('camera_yaw'),
                       LaunchConfiguration('camera_pitch'), LaunchConfiguration('camera_roll'),
                       'velodyne', 'camera_link'],
            condition=camera_condition),
        LogInfo(
            msg='velodyne->camera_link TF disabled: camera extrinsics REQUIRES_CALIBRATION',
            condition=UnlessCondition(LaunchConfiguration('camera_extrinsics_configured'))),
    ])
