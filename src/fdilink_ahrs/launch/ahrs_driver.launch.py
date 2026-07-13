#!/usr/bin/env python3

"""
> ros2 launch fdilink_ahrs ahrs_driver.launch.py
> ros2 topic echo /imu --field orientation
"""

from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node


def generate_launch_description():
    ahrs_driver = Node(
        package="fdilink_ahrs",
        executable="ahrs_driver_node",
        name="ahrs_driver",
        parameters=[{
            'if_debug_': False,
            'serial_port_': '/dev/ttyUSB0',
            'serial_baud_': 921600,
            'imu_topic': '/imu',  # remap imu:=trunk_imu
            'imu_frame_id_': 'imu_link',  # gyro_link:=world
            'mag_pose_2d_topic': '/mag_pose_2d',
            'Magnetic_topic': '/magnetic',
            'Euler_angles_topic': '/euler_angles',
            'gps_topic': '/gps/fix',
            'twist_topic': '/system_speed',
            'NED_odom_topic': '/ned_odometry',
            'device_type_': 1
        }],
        output="screen"
    )

    # imu_tf = Node(
    #     package="fdilink_ahrs",#ttyUSB0
    #     executable="imu_tf_node",
    #     parameters=[{
    #         'imu_topic': '/imu',
    #         'world_frame_id': '/world',
    #         'imu_frame_id': '/gyro_link',
    #         'position_x': 1,
    #         'position_y': 1,
    #         'position_z': 1,
    #     }],
    # )

    # load target step by step
    # load_sensors = TimerAction(
    #     period=0.0,
    #     actions=[ahrs_driver]
    #     # actions=[ahrs_driver, imu_tf]
    # )

    ld = LaunchDescription([ahrs_driver])

    return ld
