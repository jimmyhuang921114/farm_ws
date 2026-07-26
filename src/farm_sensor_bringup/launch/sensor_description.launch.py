from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch.substitutions import FindExecutable


def generate_launch_description() -> LaunchDescription:
    use_sim_time = LaunchConfiguration("use_sim_time")

    publish_livox_imu_tf = LaunchConfiguration(
        "publish_livox_imu_tf"
    )

    publish_camera_optical_tf = LaunchConfiguration(
        "publish_camera_optical_tf"
    )

    xacro_file = PathJoinSubstitution([
        FindPackageShare("farm_sensor_bringup"),
        "urdf",
        "sensor_rig.urdf.xacro",
    ])

    robot_description = ParameterValue(
        Command([
            FindExecutable(name="xacro"),
            " ",
            xacro_file,
            " ",
            "publish_livox_imu_tf:=",
            publish_livox_imu_tf,
            " ",
            "publish_camera_optical_tf:=",
            publish_camera_optical_tf,
        ]),
        value_type=str,
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="sensor_robot_state_publisher",
        output="screen",
        parameters=[{
            "robot_description": robot_description,
            "use_sim_time": use_sim_time,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use simulation clock",
        ),

        DeclareLaunchArgument(
            "publish_livox_imu_tf",
            default_value="true",
            description=(
                "Publish livox_frame to livox_imu "
                "from the URDF"
            ),
        ),

        DeclareLaunchArgument(
            "publish_camera_optical_tf",
            default_value="false",
            description=(
                "Publish camera optical TF from URDF. "
                "Keep false when the camera driver publishes it."
            ),
        ),

        robot_state_publisher,
    ])
