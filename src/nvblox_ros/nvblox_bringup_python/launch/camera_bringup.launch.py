from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "urdf_file",
            default_value=(
                "/home/jimmy/work_ws/nvblox_ros2/src/nvblox_ros/"
                "nvblox_bringup_python/config/robot.urdf.xacro"
            ),
        ),
        DeclareLaunchArgument("run_depth_model", default_value="false"),
        DeclareLaunchArgument("run_rviz",default_value="false")

        Node(
            package = "camera",
            executable = "sensor",
            name = "sensor",
            output = "screen",
        )

        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{
                "robot_description": Command([
                    FindExecutable(name="xacro"),
                    " ",
                    LaunchConfiguration("urdf_file"),
                ]),
            }],
        ),

        Node(
            package=LaunchConfiguration("depth_model_package"),
            executable=LaunchConfiguration("depth_model_executable"),
            name="depth_model",
            output="screen",
            condition=IfCondition(LaunchConfiguration("run_depth_model")),
        ),


        Node(
            package="nvblox_ros",
            executable="nvblox_node",
            name="nvblox_node",
            output="screen",
        ),
    ])