"""Launch RQT and RViz consumers for the collection/mapping topics."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Create the collection UI launch description."""
    return LaunchDescription([
        SetEnvironmentVariable(
            name='FARM_COLLECTION_UI_ONLY_DEFAULT',
            value=LaunchConfiguration('ui_only_default'),
        ),
        DeclareLaunchArgument('start_rqt', default_value='true'),
        DeclareLaunchArgument('start_rviz', default_value='true'),
        DeclareLaunchArgument('start_image_view', default_value='false'),
        DeclareLaunchArgument('ui_only_default', default_value='true'),
        DeclareLaunchArgument(
            'rviz_config',
            default_value='/workspace/farm_ws/rviz/collection.rviz',
        ),
        Node(
            package='rqt_gui',
            executable='rqt_gui',
            arguments=[
                '--standalone',
                'collection_rqt_panel.panel.CollectionPanel',
            ],
            condition=IfCondition(LaunchConfiguration('start_rqt')),
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', LaunchConfiguration('rviz_config')],
            condition=IfCondition(LaunchConfiguration('start_rviz')),
        ),
        Node(
            package='rqt_image_view',
            executable='rqt_image_view',
            condition=IfCondition(LaunchConfiguration('start_image_view')),
        ),
    ])
