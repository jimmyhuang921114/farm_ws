from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # -------------------------
    # Launch arguments
    # -------------------------
    device_ip = LaunchConfiguration('device_ip')
    port = LaunchConfiguration('port')
    lidar_frame = LaunchConfiguration('lidar_frame')
    base_frame = LaunchConfiguration('base_frame')
    imu_frame = LaunchConfiguration('imu_frame')

    calibration = LaunchConfiguration('calibration')
    min_range = LaunchConfiguration('min_range')
    max_range = LaunchConfiguration('max_range')
    organize_cloud = LaunchConfiguration('organize_cloud')

    imu_port = LaunchConfiguration('imu_port')
    imu_baud = LaunchConfiguration('imu_baud')
    imu_topic = LaunchConfiguration('imu_topic')

    auto_setup = LaunchConfiguration('auto_setup')
    auto_network = LaunchConfiguration('auto_network')
    auto_imu_permission = LaunchConfiguration('auto_imu_permission')
    net_iface = LaunchConfiguration('net_iface')
    host_lidar_ip_cidr = LaunchConfiguration('host_lidar_ip_cidr')

    base_to_lidar_x = LaunchConfiguration('base_to_lidar_x')
    base_to_lidar_y = LaunchConfiguration('base_to_lidar_y')
    base_to_lidar_z = LaunchConfiguration('base_to_lidar_z')
    base_to_lidar_roll = LaunchConfiguration('base_to_lidar_roll')
    base_to_lidar_pitch = LaunchConfiguration('base_to_lidar_pitch')
    base_to_lidar_yaw = LaunchConfiguration('base_to_lidar_yaw')

    lidar_to_imu_x = LaunchConfiguration('lidar_to_imu_x')
    lidar_to_imu_y = LaunchConfiguration('lidar_to_imu_y')
    lidar_to_imu_z = LaunchConfiguration('lidar_to_imu_z')
    lidar_to_imu_roll = LaunchConfiguration('lidar_to_imu_roll')
    lidar_to_imu_pitch = LaunchConfiguration('lidar_to_imu_pitch')
    lidar_to_imu_yaw = LaunchConfiguration('lidar_to_imu_yaw')

    # -------------------------
    # Preflight setup
    # -------------------------
    setup_imu_permission = ExecuteProcess(
        condition=IfCondition(auto_imu_permission),
        cmd=[
            'bash', '-lc',
            'echo "[preflight] chmod IMU port: $0"; '
            'if [ -e "$0" ]; then '
            '  sudo chmod 666 "$0" || true; '
            '  ls -l "$0"; '
            'else '
            '  echo "[preflight][WARN] IMU port not found: $0"; '
            'fi',
            imu_port,
        ],
        output='screen',
    )

    setup_lidar_network = ExecuteProcess(
        condition=IfCondition(auto_network),
        cmd=[
            'bash', '-lc',
            'echo "[preflight] setup lidar network"; '
            'echo "[preflight] iface=$0 host_ip=$1 device_ip=$2"; '
            'if ip link show "$0" >/dev/null 2>&1; then '
            '  sudo ip link set "$0" up || true; '
            '  if ! ip -4 addr show dev "$0" | grep -q "${1%/*}"; then '
            '    sudo ip addr add "$1" dev "$0" || true; '
            '  fi; '
            '  ip -br addr show "$0"; '
            '  ping -c 1 -W 1 "$2" || true; '
            'else '
            '  echo "[preflight][WARN] network iface not found: $0"; '
            '  ip -br addr || true; '
            'fi',
            net_iface,
            host_lidar_ip_cidr,
            device_ip,
        ],
        output='screen',
    )

    # -------------------------
    # Nodes
    # -------------------------
    velodyne_driver = Node(
        package='velodyne_driver',
        executable='velodyne_driver_node',
        name='velodyne_driver_node',
        output='screen',
        parameters=[{
            'device_ip': device_ip,
            'port': port,
            'frame_id': lidar_frame,
            'model': 'VLP16',
            'rpm': 600.0,
            'gps_time': False,
            'time_offset': 0.0,
            'enabled': True,
            'read_once': False,
            'read_fast': False,
            'repeat_delay': 0.0,
            'timestamp_first_packet': False,
        }],
    )

    velodyne_transform = Node(
        package='velodyne_pointcloud',
        executable='velodyne_transform_node',
        name='velodyne_transform_node',
        output='screen',
        parameters=[{
            'calibration': calibration,
            'model': 'VLP16',
            'min_range': min_range,
            'max_range': max_range,
            'target_frame': lidar_frame,
            'fixed_frame': lidar_frame,
            'organize_cloud': organize_cloud,
        }],
    )

    imu_driver = Node(
        package='fdilink_ahrs',
        executable='ahrs_driver_node',
        name='fdilink_ahrs_driver',
        output='screen',
        parameters=[{
            'serial_port_': imu_port,
            'serial_baud_': imu_baud,
            'imu_topic': imu_topic,
            'frame_id': imu_frame,
        }],
    )

    base_to_velodyne_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_velodyne_tf',
        output='screen',
        arguments=[
            '--x', base_to_lidar_x,
            '--y', base_to_lidar_y,
            '--z', base_to_lidar_z,
            '--roll', base_to_lidar_roll,
            '--pitch', base_to_lidar_pitch,
            '--yaw', base_to_lidar_yaw,
            '--frame-id', base_frame,
            '--child-frame-id', lidar_frame,
        ],
    )

    velodyne_to_imu_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='velodyne_to_imu_tf',
        output='screen',
        arguments=[
            '--x', lidar_to_imu_x,
            '--y', lidar_to_imu_y,
            '--z', lidar_to_imu_z,
            '--roll', lidar_to_imu_roll,
            '--pitch', lidar_to_imu_pitch,
            '--yaw', lidar_to_imu_yaw,
            '--frame-id', lidar_frame,
            '--child-frame-id', imu_frame,
        ],
    )

    delayed_nodes = TimerAction(
        period=2.0,
        actions=[
            velodyne_driver,
            velodyne_transform,
            imu_driver,
            base_to_velodyne_tf,
            velodyne_to_imu_tf,
        ],
    )

    return LaunchDescription([
        # -------------------------
        # Main args
        # -------------------------
        DeclareLaunchArgument('device_ip', default_value='192.168.1.201'),
        DeclareLaunchArgument('port', default_value='2368'),
        DeclareLaunchArgument('lidar_frame', default_value='velodyne'),
        DeclareLaunchArgument('base_frame', default_value='base_link'),
        DeclareLaunchArgument('imu_frame', default_value='imu_link'),

        DeclareLaunchArgument(
            'calibration',
            default_value='/opt/ros/humble/share/velodyne_pointcloud/params/VLP16db.yaml'
        ),
        DeclareLaunchArgument('min_range', default_value='0.3'),
        DeclareLaunchArgument('max_range', default_value='30.0'),
        DeclareLaunchArgument('organize_cloud', default_value='false'),

        DeclareLaunchArgument('imu_port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('imu_baud', default_value='921600'),
        DeclareLaunchArgument('imu_topic', default_value='/imu'),

        # -------------------------
        # Auto setup args
        # -------------------------
        DeclareLaunchArgument('auto_setup', default_value='true'),
        DeclareLaunchArgument('auto_network', default_value='true'),
        DeclareLaunchArgument('auto_imu_permission', default_value='true'),

        # 依你的前面設定，USB Ethernet 常見是 enx00e04c521538
        DeclareLaunchArgument('net_iface', default_value='enx00e04c521538'),

        # host 端 LiDAR 網段 IP。VLP-16 是 192.168.1.201，所以 host 用 192.168.1.x
        DeclareLaunchArgument('host_lidar_ip_cidr', default_value='192.168.1.10/24'),

        # -------------------------
        # Static TF args
        # -------------------------
        DeclareLaunchArgument('base_to_lidar_x', default_value='0.0'),
        DeclareLaunchArgument('base_to_lidar_y', default_value='0.0'),
        DeclareLaunchArgument('base_to_lidar_z', default_value='0.0'),
        DeclareLaunchArgument('base_to_lidar_roll', default_value='0.0'),
        DeclareLaunchArgument('base_to_lidar_pitch', default_value='0.0'),
        DeclareLaunchArgument('base_to_lidar_yaw', default_value='0.0'),

        DeclareLaunchArgument('lidar_to_imu_x', default_value='0.0'),
        DeclareLaunchArgument('lidar_to_imu_y', default_value='0.0'),
        DeclareLaunchArgument('lidar_to_imu_z', default_value='0.07'),
        DeclareLaunchArgument('lidar_to_imu_roll', default_value='0.0'),
        DeclareLaunchArgument('lidar_to_imu_pitch', default_value='0.0'),
        DeclareLaunchArgument('lidar_to_imu_yaw', default_value='0.0'),

        setup_imu_permission,
        setup_lidar_network,
        delayed_nodes,
    ])
