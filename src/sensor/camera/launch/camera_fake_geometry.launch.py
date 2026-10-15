"""Start RGB/depth with temporary intrinsics and a fixed camera pose."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    video_device = LaunchConfiguration('video_device')
    horizontal_fov = LaunchConfiguration('horizontal_fov_deg')
    checkpoint = LaunchConfiguration('checkpoint')

    def static_tf(name, parent, child, *pose):
        return Node(
            package='tf2_ros', executable='static_transform_publisher',
            name=name, output='screen',
            arguments=[
                '--x', str(pose[0]), '--y', str(pose[1]), '--z', str(pose[2]),
                '--qx', str(pose[3]), '--qy', str(pose[4]),
                '--qz', str(pose[5]), '--qw', str(pose[6]),
                '--frame-id', parent, '--child-frame-id', child,
            ],
        )

    return LaunchDescription([
        DeclareLaunchArgument('video_device', default_value='/dev/video1'),
        DeclareLaunchArgument('horizontal_fov_deg', default_value='70.0'),
        DeclareLaunchArgument('checkpoint', default_value=PathJoinSubstitution([
            FindPackageShare('camera'), 'model', 'best.pt'])),
        Node(
            package='camera', executable='camera', name='camera', output='screen',
            parameters=[{
                'video_device': video_device,
                'checkpoint': checkpoint,
                'frame_id': 'camera_optical_frame',
                'approximate_horizontal_fov_deg': ParameterValue(horizontal_fov, value_type=float),
            }],
        ),
        static_tf('map_to_base', 'map', 'base_link', 0, 0, 0, 0, 0, 0, 1),
        static_tf('base_to_camera', 'base_link', 'camera_link', 0.20, 0, 0.35, 0, 0, 0, 1),
        static_tf('camera_to_optical', 'camera_link', 'camera_optical_frame',
                  0, 0, 0, -0.5, 0.5, -0.5, 0.5),
    ])
