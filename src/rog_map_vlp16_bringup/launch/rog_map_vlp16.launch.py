import os
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _write_config(context):
    cloud_topic = LaunchConfiguration("cloud_topic").perform(context)
    odom_topic = LaunchConfiguration("odom_topic").perform(context)
    frame_id = LaunchConfiguration("frame_id").perform(context)
    share_dir = get_package_share_directory("rog_map_vlp16_bringup")
    rviz_config = os.path.join(share_dir, "rviz", "rog_map_vlp16.rviz")

    config_text = f"""rog_map:
  ros_callback:
    enable: true
    cloud_topic: {cloud_topic}
    odom_topic: {odom_topic}
    odom_timeout: 2.0

  visualization:
    enable: true
    frame_id: {frame_id}
    time_rate: 2.0
    frame_rate: 0
    range: [30.0, 30.0, 6.0]
    pub_unknown_map_en: false
    use_dynamic_reconfigure: false

  esdf:
    enable: true
    resolution: 0.25
    local_update_box: [20.0, 20.0, 4.0]

  load_pcd_en: false
  map_sliding:
    enable: true
    threshold: 2.0
  fix_map_origin: [0.0, 0.0, 0.0]
  frontier_extraction_en: false

  resolution: 0.20
  inflation_resolution: 0.30
  inflation_step: 1
  unk_inflation_en: false
  unk_inflation_step: 1
  intensity_thresh: -1
  map_size: [40.0, 40.0, 8.0]
  point_filt_num: 2
  virtual_ground_height: -2.0
  virtual_ceil_height: 4.0

  raycasting:
    enable: true
    batch_update_size: 1
    unk_thresh: 0.70
    p_hit: 0.70
    p_miss: 0.70
    p_min: 0.12
    p_max: 0.97
    p_occ: 0.80
    p_free: 0.30
    ray_range: [0.5, 25.0]
    local_update_box: [30.0, 30.0, 6.0]
"""
    cfg = tempfile.NamedTemporaryFile(
        mode="w",
        prefix="rog_map_vlp16_",
        suffix=".yaml",
        delete=False,
    )
    cfg.write(config_text)
    cfg.close()

    return [
        Node(
            package="rog_map_vlp16_bringup",
            executable="rog_map_vlp16_node",
            name="rog_map_vlp16",
            output="screen",
            parameters=[{"config_file": cfg.name}],
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2_rog_map_vlp16",
            output="screen",
            arguments=["-d", rviz_config],
            condition=IfCondition(LaunchConfiguration("rviz")),
        ),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("cloud_topic", default_value="/cloud_registered"),
            DeclareLaunchArgument("odom_topic", default_value="/kiss/odometry"),
            DeclareLaunchArgument("frame_id", default_value="odom"),
            DeclareLaunchArgument("rviz", default_value="true"),
            OpaqueFunction(function=_write_config),
        ]
    )
