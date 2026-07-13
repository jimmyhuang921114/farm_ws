from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='cloudpoint_process',
            executable='open3d_occupancy_block_node',
            name='open3d_occupancy_block_node',
            output='screen',
            parameters=[{
                'cloud_topic': '/velodyne_points',
                'map_frame': 'velodyne',

                # 前處理
                'voxel_size': 0.10,
                'max_range': 15.0,
                'min_range': 0.3,
                'min_z': -2.0,
                'max_z': 2.0,
                'max_input_points': 40000,

                # 離群濾波
                'enable_outlier_filter': True,

                # options: statistical / radius / both / none
                'outlier_filter_method': 'statistical',

                # Statistical outlier removal
                'stat_nb_neighbors': 20,
                'stat_std_ratio': 2.0,

                # Radius outlier removal
                'radius_nb_points': 4,
                'radius_search_radius': 0.35,

                # ray casting
                'ray_step_factor': 0.7,

                # log-odds
                'logodds_occ': 0.85,
                'logodds_free': -0.40,
                'logodds_min': -3.5,
                'logodds_max': 3.5,
                'occ_threshold': 0.8,
                'free_threshold': -0.8,

                # publish
                'publish_every_n_frames': 1,
                'max_publish_voxels': 200000,

                # projected map
                'project_min_z': -0.5,
                'project_max_z': 2.0,
                'map_min_x': -20.0,
                'map_max_x': 20.0,
                'map_min_y': -20.0,
                'map_max_y': 20.0,

                # topics
                'occupied_points_topic': '/open3d_occupancy/occupied_points',
                'free_points_topic': '/open3d_occupancy/free_points',
                'occupied_blocks_topic': '/open3d_occupancy/occupied_blocks',
                'free_blocks_topic': '/open3d_occupancy/free_blocks',
                'projected_map_topic': '/open3d_occupancy/projected_map',
                'filtered_points_topic': '/open3d_occupancy/filtered_points',

                # RViz 建議先關掉 free blocks
                'publish_free_blocks': False,
                'publish_filtered_points': True,
            }]
        )
    ])
