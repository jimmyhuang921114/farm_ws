from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    camera_node = Node(
        package='sensor_bringup',
        executable='open_camera',
        name='decxin_camera',
        output='screen',
        parameters=[{
            'vendor_id': '1bcf',
            'product_id': '2cd1',

            'image_topic': '/decxin_camera/image_raw',
            'frame_id': 'decxin_camera_link',

            'width': 1280,
            'height': 720,
            'fps': 30.0,
            'fourcc': 'MJPG',

            'reconnect_interval': 1.0,
            'max_read_failures': 10,
        }],
    )

    imu_transform_node = Node(
        package='sensor_bringup',
        executable='imu_transform',
        name='livox_imu_transform',
        output='screen',
        parameters=[{
            'input_topic': '/livox/imu',
            'output_topic': '/livox/imu_base',

            'target_frame': 'base_link',
            'source_frame_override': '',

            # Input magnitude is approximately 1.0 at rest,
            # so it is probably measured in g.
            'accel_scale': 9.80665,

            'use_latest_tf': True,
            'tf_timeout': 0.1,

            # Livox currently outputs identity orientation,
            # not a real attitude estimate.
            'force_orientation_unavailable': True,
        }],
    )

    return LaunchDescription([
        camera_node,
        imu_transform_node,
    ])
