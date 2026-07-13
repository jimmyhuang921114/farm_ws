from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='lidar_rgb_fusion',
            executable='lidar_rgb_fusion_node',
            name='lidar_rgb_fusion_node',
            output='screen',
            parameters=[{
                'lidar_topic': '/velodyne_points',
                'image_topic': '/camera/camera/color/image_raw',
                'camera_info_topic': '/camera/camera/color/camera_info',
                'output_cloud_topic': '/velodyne_colored_points',
                'debug_image_topic': '/lidar_rgb_fusion/debug_image',
                'target_camera_frame': 'camera_color_optical_frame',
                'min_depth': 0.1,
                'max_depth': 30.0,
                'sync_slop': 0.2,
                'max_points': 30000,
                'debug_point_radius': 2,
            }],
        ),
    ])
