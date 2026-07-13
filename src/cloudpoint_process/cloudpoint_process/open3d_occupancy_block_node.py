#!/usr/bin/env python3
import math
import numpy as np

import rclpy
from rclpy.node import Node

from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from nav_msgs.msg import OccupancyGrid, MapMetaData
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point

try:
    import open3d as o3d
    HAS_OPEN3D = True
except Exception:
    HAS_OPEN3D = False


def voxel_key(point, voxel_size):
    return (
        int(math.floor(point[0] / voxel_size)),
        int(math.floor(point[1] / voxel_size)),
        int(math.floor(point[2] / voxel_size)),
    )


def key_to_center(key, voxel_size):
    return np.array([
        (key[0] + 0.5) * voxel_size,
        (key[1] + 0.5) * voxel_size,
        (key[2] + 0.5) * voxel_size,
    ], dtype=np.float32)


def make_xyz_cloud(header, points):
    fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    pts = [(float(p[0]), float(p[1]), float(p[2])) for p in points]
    return point_cloud2.create_cloud(header, fields, pts)


def make_cube_list_marker(header, points_np, ns, marker_id, voxel_size, r, g, b, a):
    marker = Marker()
    marker.header = header
    marker.ns = ns
    marker.id = marker_id
    marker.type = Marker.CUBE_LIST
    marker.action = Marker.ADD
    marker.pose.orientation.w = 1.0

    marker.scale.x = float(voxel_size)
    marker.scale.y = float(voxel_size)
    marker.scale.z = float(voxel_size)

    marker.color.r = float(r)
    marker.color.g = float(g)
    marker.color.b = float(b)
    marker.color.a = float(a)

    marker.points = []
    for p in points_np:
        pt = Point()
        pt.x = float(p[0])
        pt.y = float(p[1])
        pt.z = float(p[2])
        marker.points.append(pt)

    return marker


class Open3DOccupancyBlockNode(Node):
    def __init__(self):
        super().__init__('open3d_occupancy_block_node')

        self.declare_parameter('cloud_topic', '/velodyne_points')
        self.declare_parameter('map_frame', 'velodyne')

        # 前處理
        self.declare_parameter('voxel_size', 0.10)
        self.declare_parameter('max_range', 15.0)
        self.declare_parameter('min_range', 0.3)
        self.declare_parameter('min_z', -10.0)
        self.declare_parameter('max_z', 10.0)
        self.declare_parameter('max_input_points', 40000)

        # 離群濾波
        self.declare_parameter('enable_outlier_filter', True)
        self.declare_parameter('outlier_filter_method', 'statistical')  # statistical / radius / both / none

        # Statistical outlier removal
        self.declare_parameter('stat_nb_neighbors', 20)
        self.declare_parameter('stat_std_ratio', 2.0)

        # Radius outlier removal
        self.declare_parameter('radius_nb_points', 4)
        self.declare_parameter('radius_search_radius', 0.35)

        # Ray casting
        self.declare_parameter('ray_step_factor', 0.7)

        # log-odds
        self.declare_parameter('logodds_occ', 0.85)
        self.declare_parameter('logodds_free', -0.40)
        self.declare_parameter('logodds_min', -3.5)
        self.declare_parameter('logodds_max', 3.5)
        self.declare_parameter('occ_threshold', 0.8)
        self.declare_parameter('free_threshold', -0.8)

        # publish
        self.declare_parameter('publish_every_n_frames', 1)
        self.declare_parameter('max_publish_voxels', 200000)

        # 2D projection
        self.declare_parameter('project_min_z', -0.5)
        self.declare_parameter('project_max_z', 2.0)
        self.declare_parameter('map_min_x', -20.0)
        self.declare_parameter('map_max_x', 20.0)
        self.declare_parameter('map_min_y', -20.0)
        self.declare_parameter('map_max_y', 20.0)

        # topics
        self.declare_parameter('occupied_points_topic', '/open3d_occupancy/occupied_points')
        self.declare_parameter('free_points_topic', '/open3d_occupancy/free_points')
        self.declare_parameter('occupied_blocks_topic', '/open3d_occupancy/occupied_blocks')
        self.declare_parameter('free_blocks_topic', '/open3d_occupancy/free_blocks')
        self.declare_parameter('projected_map_topic', '/open3d_occupancy/projected_map')
        self.declare_parameter('filtered_points_topic', '/open3d_occupancy/filtered_points')
        self.declare_parameter('publish_free_blocks', False)
        self.declare_parameter('publish_filtered_points', True)

        self.cloud_topic = self.get_parameter('cloud_topic').value
        self.map_frame = self.get_parameter('map_frame').value

        self.voxel_size = float(self.get_parameter('voxel_size').value)
        self.max_range = float(self.get_parameter('max_range').value)
        self.min_range = float(self.get_parameter('min_range').value)
        self.min_z = float(self.get_parameter('min_z').value)
        self.max_z = float(self.get_parameter('max_z').value)
        self.max_input_points = int(self.get_parameter('max_input_points').value)

        self.enable_outlier_filter = bool(self.get_parameter('enable_outlier_filter').value)
        self.outlier_filter_method = str(self.get_parameter('outlier_filter_method').value)

        self.stat_nb_neighbors = int(self.get_parameter('stat_nb_neighbors').value)
        self.stat_std_ratio = float(self.get_parameter('stat_std_ratio').value)

        self.radius_nb_points = int(self.get_parameter('radius_nb_points').value)
        self.radius_search_radius = float(self.get_parameter('radius_search_radius').value)

        self.ray_step_factor = float(self.get_parameter('ray_step_factor').value)

        self.logodds_occ = float(self.get_parameter('logodds_occ').value)
        self.logodds_free = float(self.get_parameter('logodds_free').value)
        self.logodds_min = float(self.get_parameter('logodds_min').value)
        self.logodds_max = float(self.get_parameter('logodds_max').value)
        self.occ_threshold = float(self.get_parameter('occ_threshold').value)
        self.free_threshold = float(self.get_parameter('free_threshold').value)

        self.publish_every_n_frames = int(self.get_parameter('publish_every_n_frames').value)
        self.max_publish_voxels = int(self.get_parameter('max_publish_voxels').value)

        self.project_min_z = float(self.get_parameter('project_min_z').value)
        self.project_max_z = float(self.get_parameter('project_max_z').value)
        self.map_min_x = float(self.get_parameter('map_min_x').value)
        self.map_max_x = float(self.get_parameter('map_max_x').value)
        self.map_min_y = float(self.get_parameter('map_min_y').value)
        self.map_max_y = float(self.get_parameter('map_max_y').value)

        self.occupied_points_topic = self.get_parameter('occupied_points_topic').value
        self.free_points_topic = self.get_parameter('free_points_topic').value
        self.occupied_blocks_topic = self.get_parameter('occupied_blocks_topic').value
        self.free_blocks_topic = self.get_parameter('free_blocks_topic').value
        self.projected_map_topic = self.get_parameter('projected_map_topic').value
        self.filtered_points_topic = self.get_parameter('filtered_points_topic').value
        self.publish_free_blocks = bool(self.get_parameter('publish_free_blocks').value)
        self.publish_filtered_points = bool(self.get_parameter('publish_filtered_points').value)

        self.voxel_logodds = {}
        self.frame_count = 0

        self.sub = self.create_subscription(PointCloud2, self.cloud_topic, self.cloud_callback, 10)

        self.filtered_points_pub = self.create_publisher(PointCloud2, self.filtered_points_topic, 10)
        self.occ_points_pub = self.create_publisher(PointCloud2, self.occupied_points_topic, 10)
        self.free_points_pub = self.create_publisher(PointCloud2, self.free_points_topic, 10)
        self.occ_blocks_pub = self.create_publisher(MarkerArray, self.occupied_blocks_topic, 10)
        self.free_blocks_pub = self.create_publisher(MarkerArray, self.free_blocks_topic, 10)
        self.map_pub = self.create_publisher(OccupancyGrid, self.projected_map_topic, 10)

        self.get_logger().info('open3d_occupancy_block_node started')
        self.get_logger().info(f'cloud_topic: {self.cloud_topic}')
        self.get_logger().info(f'map_frame: {self.map_frame}')
        self.get_logger().info(f'voxel_size: {self.voxel_size}')
        self.get_logger().info(f'Open3D available: {HAS_OPEN3D}')
        self.get_logger().info(f'outlier filter: enable={self.enable_outlier_filter}, method={self.outlier_filter_method}')
        self.get_logger().info(f'statistical: nb_neighbors={self.stat_nb_neighbors}, std_ratio={self.stat_std_ratio}')
        self.get_logger().info(f'radius: nb_points={self.radius_nb_points}, radius={self.radius_search_radius}')

    def cloud_callback(self, msg):
        self.frame_count += 1

        points = []
        raw_count = 0
        range_filtered_count = 0

        for p in point_cloud2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True):
            raw_count += 1

            x, y, z = float(p[0]), float(p[1]), float(p[2])
            dist = math.sqrt(x * x + y * y + z * z)

            if dist < self.min_range or dist > self.max_range:
                continue
            if z < self.min_z or z > self.max_z:
                continue

            points.append([x, y, z])
            range_filtered_count += 1

            if len(points) >= self.max_input_points:
                break

        if not points:
            self.get_logger().warn('No valid input points after range/z filter', throttle_duration_sec=2.0)
            return

        points_np = np.asarray(points, dtype=np.float32)

        # 1. 離群點濾波
        before_outlier = points_np.shape[0]
        points_np = self.open3d_outlier_filter(points_np)
        after_outlier = points_np.shape[0]

        if points_np.shape[0] == 0:
            self.get_logger().warn('No valid points after outlier filter', throttle_duration_sec=2.0)
            return

        # 2. Voxel downsample
        before_downsample = points_np.shape[0]
        points_np = self.open3d_downsample(points_np)
        after_downsample = points_np.shape[0]

        if self.publish_filtered_points:
            header = Header()
            header.stamp = msg.header.stamp
            header.frame_id = self.map_frame
            self.filtered_points_pub.publish(make_xyz_cloud(header, points_np))

        # 3. Ray casting integrate
        self.integrate_scan(points_np)

        self.get_logger().info(
            f'preprocess raw={raw_count}, range_z={range_filtered_count}, '
            f'outlier {before_outlier}->{after_outlier}, '
            f'downsample {before_downsample}->{after_downsample}',
            throttle_duration_sec=1.0,
        )

        if self.frame_count % self.publish_every_n_frames == 0:
            self.publish_all(msg.header.stamp)

    def make_o3d_pcd(self, points_np):
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points_np.astype(np.float64))
        return pcd

    def open3d_outlier_filter(self, points_np):
        if not self.enable_outlier_filter:
            return points_np

        if self.outlier_filter_method.lower() in ['none', 'off', 'false']:
            return points_np

        if not HAS_OPEN3D:
            self.get_logger().warn('Open3D not available, skip outlier filter', throttle_duration_sec=2.0)
            return points_np

        if points_np.shape[0] < 10:
            return points_np

        try:
            method = self.outlier_filter_method.lower()
            pcd = self.make_o3d_pcd(points_np)

            if method in ['statistical', 'stat', 'both']:
                pcd, ind = pcd.remove_statistical_outlier(
                    nb_neighbors=self.stat_nb_neighbors,
                    std_ratio=self.stat_std_ratio,
                )
                if len(ind) == 0:
                    return points_np

            if method in ['radius', 'rad', 'both']:
                pcd, ind = pcd.remove_radius_outlier(
                    nb_points=self.radius_nb_points,
                    radius=self.radius_search_radius,
                )
                if len(ind) == 0:
                    return points_np

            return np.asarray(pcd.points, dtype=np.float32)

        except Exception as exc:
            self.get_logger().warn(f'Open3D outlier filter failed: {exc}', throttle_duration_sec=2.0)
            return points_np

    def open3d_downsample(self, points_np):
        if not HAS_OPEN3D:
            return points_np

        if points_np.shape[0] == 0:
            return points_np

        # GPU Open3D tensor voxel downsample
        try:
            if o3d.core.cuda.is_available():
                device = o3d.core.Device("CUDA:0")
                t_points = o3d.core.Tensor(
                    points_np.astype(np.float32),
                    dtype=o3d.core.Dtype.Float32,
                    device=device,
                )

                t_pcd = o3d.t.geometry.PointCloud(device)
                t_pcd.point.positions = t_points
                t_pcd = t_pcd.voxel_down_sample(self.voxel_size)

                out = t_pcd.point.positions.cpu().numpy()
                return out.astype(np.float32)

        except Exception as exc:
            self.get_logger().warn(
                f'Open3D GPU downsample failed, fallback to CPU: {exc}',
                throttle_duration_sec=2.0,
            )

        # CPU fallback
        try:
            pcd = self.make_o3d_pcd(points_np)
            pcd = pcd.voxel_down_sample(self.voxel_size)
            return np.asarray(pcd.points, dtype=np.float32)
        except Exception as exc:
            self.get_logger().warn(f'Open3D CPU downsample failed: {exc}', throttle_duration_sec=2.0)
            return points_np

    def update_logodds(self, key, delta):
        old = self.voxel_logodds.get(key, 0.0)
        new = old + delta
        new = max(self.logodds_min, min(self.logodds_max, new))
        self.voxel_logodds[key] = new

    def integrate_scan(self, points_np):
        origin = np.array([0.0, 0.0, 0.0], dtype=np.float32)
        step = max(self.voxel_size * self.ray_step_factor, self.voxel_size * 0.25)

        for p in points_np:
            distance = float(np.linalg.norm(p))
            if distance < self.min_range:
                continue

            direction = p / max(distance, 1e-6)
            n_steps = int(distance / step)
            endpoint_key = voxel_key(p, self.voxel_size)

            last_key = None
            for i in range(1, max(1, n_steps)):
                q = origin + direction * (i * step)
                k = voxel_key(q, self.voxel_size)

                if k == endpoint_key:
                    continue

                if k != last_key:
                    self.update_logodds(k, self.logodds_free)
                    last_key = k

            self.update_logodds(endpoint_key, self.logodds_occ)

        self.get_logger().info(
            f'integrated input={len(points_np)}, voxels={len(self.voxel_logodds)}',
            throttle_duration_sec=1.0,
        )

    def get_state_points(self):
        occupied = []
        free = []

        count = 0
        for k, logodds in self.voxel_logodds.items():
            if count >= self.max_publish_voxels:
                break

            if logodds >= self.occ_threshold:
                occupied.append(key_to_center(k, self.voxel_size))
                count += 1
            elif logodds <= self.free_threshold:
                free.append(key_to_center(k, self.voxel_size))
                count += 1

        occupied = np.vstack(occupied).astype(np.float32) if occupied else np.zeros((0, 3), dtype=np.float32)
        free = np.vstack(free).astype(np.float32) if free else np.zeros((0, 3), dtype=np.float32)
        return occupied, free

    def publish_all(self, stamp):
        occupied, free = self.get_state_points()

        header = Header()
        header.stamp = stamp
        header.frame_id = self.map_frame

        self.occ_points_pub.publish(make_xyz_cloud(header, occupied))
        self.free_points_pub.publish(make_xyz_cloud(header, free))
        self.publish_markers(header, occupied, free)
        self.map_pub.publish(self.build_projected_map(stamp))

        self.get_logger().info(
            f'published occupied={occupied.shape[0]}, free={free.shape[0]}',
            throttle_duration_sec=1.0,
        )

    def publish_markers(self, header, occupied, free):
        occ_arr = MarkerArray()

        clear = Marker()
        clear.header = header
        clear.ns = 'occupied_blocks'
        clear.action = Marker.DELETEALL
        occ_arr.markers.append(clear)

        occ_arr.markers.append(
            make_cube_list_marker(
                header,
                occupied,
                'occupied_blocks',
                0,
                self.voxel_size,
                1.0,
                0.05,
                0.05,
                0.85,
            )
        )
        self.occ_blocks_pub.publish(occ_arr)

        if self.publish_free_blocks:
            free_arr = MarkerArray()

            clear_free = Marker()
            clear_free.header = header
            clear_free.ns = 'free_blocks'
            clear_free.action = Marker.DELETEALL
            free_arr.markers.append(clear_free)

            free_arr.markers.append(
                make_cube_list_marker(
                    header,
                    free,
                    'free_blocks',
                    0,
                    self.voxel_size,
                    0.1,
                    0.4,
                    1.0,
                    0.18,
                )
            )
            self.free_blocks_pub.publish(free_arr)

    def build_projected_map(self, stamp):
        res = self.voxel_size
        width = int(math.ceil((self.map_max_x - self.map_min_x) / res))
        height = int(math.ceil((self.map_max_y - self.map_min_y) / res))

        grid = np.full((height, width), -1, dtype=np.int8)

        for k, logodds in self.voxel_logodds.items():
            center = key_to_center(k, self.voxel_size)
            x, y, z = float(center[0]), float(center[1]), float(center[2])

            if z < self.project_min_z or z > self.project_max_z:
                continue
            if x < self.map_min_x or x >= self.map_max_x:
                continue
            if y < self.map_min_y or y >= self.map_max_y:
                continue

            gx = int((x - self.map_min_x) / res)
            gy = int((y - self.map_min_y) / res)

            if gx < 0 or gx >= width or gy < 0 or gy >= height:
                continue

            if logodds <= self.free_threshold:
                if grid[gy, gx] != 100:
                    grid[gy, gx] = 0
            elif logodds >= self.occ_threshold:
                grid[gy, gx] = 100

        msg = OccupancyGrid()
        msg.header.stamp = stamp
        msg.header.frame_id = self.map_frame
        msg.info = MapMetaData()
        msg.info.resolution = float(res)
        msg.info.width = int(width)
        msg.info.height = int(height)
        msg.info.origin.position.x = float(self.map_min_x)
        msg.info.origin.position.y = float(self.map_min_y)
        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation.w = 1.0
        msg.data = grid.flatten(order='C').astype(np.int8).tolist()
        return msg


def main(args=None):
    rclpy.init(args=args)
    node = Open3DOccupancyBlockNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
