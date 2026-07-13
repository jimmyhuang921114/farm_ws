from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config_file = LaunchConfiguration("config_file")
    use_sim_time = LaunchConfiguration("use_sim_time")

    return LaunchDescription([
        DeclareLaunchArgument(
            "config_file",
            default_value=(
                "/workspace/farm_ws/pointcloud_process_ws/src/"
                "FAST_LIO/config/vlp16_fdi.yaml"
            ),
            description="Absolute path to the VLP-16 + FDI IMU FAST-LIO config",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use the ROS simulation clock",
        ),
        Node(
            package="fast_lio",
            executable="fastlio_mapping",
            parameters=[config_file, {"use_sim_time": use_sim_time}],
            output="screen",
        ),
    ])
