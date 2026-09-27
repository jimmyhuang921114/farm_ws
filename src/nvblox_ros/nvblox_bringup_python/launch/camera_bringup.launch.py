from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    urdf_file = LaunchConfiguration("urdf_file")
    run_depth_model = LaunchConfiguration("run_depth_model")
    run_rviz = LaunchConfiguration("run_rviz")

    camera_frame = LaunchConfiguration("camera_frame")
    camera_x = LaunchConfiguration("camera_x")
    camera_y = LaunchConfiguration("camera_y")
    camera_z = LaunchConfiguration("camera_z")
    camera_roll = LaunchConfiguration("camera_roll")
    camera_pitch = LaunchConfiguration("camera_pitch")
    camera_yaw = LaunchConfiguration("camera_yaw")

    robot_description = Command([
        FindExecutable(name="xacro"),
        " ",
        urdf_file,
    ])

    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument(
            "urdf_file",
            default_value=(
                "/home/jimmy/work_ws/nvblox_ros2/src/nvblox_ros/"
                "nvblox_bringup_python/config/robot.urdf.xacro"
            ),
            description="Robot URDF/Xacro file",
        ),
        DeclareLaunchArgument(
            "run_depth_model",
            default_value="false",
            description="在 camera 節點中啟用 depth_estimate 推論",
        ),
        DeclareLaunchArgument(
            "run_rviz",
            default_value="false",
            description="是否啟動 RViz",
        ),

        # base_link -> camera_link 靜態 TF
        # 平移單位為公尺，旋轉單位為弧度。
        DeclareLaunchArgument("camera_frame", default_value="camera_link"),
        DeclareLaunchArgument("camera_x", default_value="0.0"),
        DeclareLaunchArgument("camera_y", default_value="0.0"),
        DeclareLaunchArgument("camera_z", default_value="0.0"),
        DeclareLaunchArgument("camera_roll", default_value="0.0"),
        DeclareLaunchArgument("camera_pitch", default_value="0.0"),
        DeclareLaunchArgument("camera_yaw", default_value="0.0"),

        # Camera node：開啟相機並發布影像
        Node(
            package="camera",
            executable="camera",
            name="camera",
            output="screen",
            parameters=[{
                "run_depth_model": ParameterValue(
                    run_depth_model,
                    value_type=bool,
                ),
                "frame_id": camera_frame,
            }],
        ),

        # Robot state publisher：發布 URDF 中定義的機器人 TF
        # Node(
        #     package="robot_state_publisher",
        #     executable="robot_state_publisher",
        #     name="robot_state_publisher",
        #     output="screen",
        #     parameters=[{
        #         "robot_description": robot_description,
        #     }],
        # ),

        # 靜態 TF：base_link -> camera_link
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="base_link_to_camera",
            output="screen",
            arguments=[
                "--x", camera_x,
                "--y", camera_y,
                "--z", camera_z,
                "--roll", camera_roll,
                "--pitch", camera_pitch,
                "--yaw", camera_yaw,
                "--frame-id", "base_link",
                "--child-frame-id", camera_frame,
            ],
        ),

        # nvblox node
        Node(
            package="nvblox_ros",
            executable="nvblox_node",
            name="nvblox_node",
            output="screen",
        ),

        # RViz（選用）
        visualization_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare("nvblox_examples_bringup"),
                    "launch",
                    "visualization",
                    "visualization.launch.py",
                ])
            ),
            launch_arguments={
                "mode": LaunchConfiguration("mode"),
                "camera": camera_mode,
                "use_foxglove_whitelist": LaunchConfiguration(
                    "use_foxglove_whitelist"
                ),
            }.items(),
        )

        actions.append(visualization_launch)
    ])