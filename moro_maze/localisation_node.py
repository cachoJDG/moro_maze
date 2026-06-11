import math

import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
import numpy as np
from nav_msgs.msg import OccupancyGrid
from nav_msgs.srv import GetMap
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan

try:
    from sklearn.neighbors import KNeighborsClassifier
except ImportError:
    class KNeighborsClassifier:
        def __init__(self, n_neighbors=1):
            self.n_neighbors = max(1, int(n_neighbors))
            self._x_train = None
            self._y_train = None

        def fit(self, x_train, y_train):
            self._x_train = np.asarray(x_train, dtype=np.float32)
            self._y_train = np.asarray(y_train, dtype=np.int8)
            return self

        def predict(self, x_query):
            x_query = np.asarray(x_query, dtype=np.float32)
            if self._x_train is None or self._y_train is None:
                raise RuntimeError('KNeighborsClassifier used before fit()')
            if x_query.size == 0:
                return np.empty((0,), dtype=np.int8)

            delta = self._x_train[None, :, :] - x_query[:, None, :]
            distances = np.sum(delta * delta, axis=2)
            neighbor_count = min(self.n_neighbors, len(self._x_train))
            nearest = np.argpartition(distances, neighbor_count - 1, axis=1)[:, :neighbor_count]
            neighbor_labels = self._y_train[nearest]
            return (np.sum(neighbor_labels, axis=1) >= (neighbor_count / 2.0)).astype(np.int8)

from moro_maze.map_utils import GridMap


class LocalisationNode(Node):
    # --- Tuning constants (hardcoded; kept simple, no ROS parameters) -------
    SCAN_TOPIC = '/scan'
    POSE_TOPIC = '/estimated_pose'
    POSE_COV_TOPIC = '/estimated_pose_cov'
    INITIAL_POSE_TOPIC = '/initialpose'
    MAP_SERVICE = '/map_server/map'
    OCCUPIED_THRESHOLD = 65       # occupancy value at/above which a cell is a wall
    SCAN_DOWNSAMPLE_STEP = 8      # use every Nth laser beam
    KNN_NEIGHBORS = 1             # k for the free/wall classifier
    MAP_SERVICE_TIMEOUT_SEC = 5.0  # [s] wait for the map service

    def __init__(self):
        super().__init__('localisation_node')

        self.grid_map = None
        self.knn_model = None
        self.scan_subscription = None
        self.map_request_future = None
        self.map_loaded = False
        self.received_scan_once = False
        self.map_array = None
        self.free_positions = np.empty((0, 2), dtype=np.float32)
        self.wall_positions = np.empty((0, 2), dtype=np.float32)
        self.knn_x = np.empty((0, 2), dtype=np.float32)
        self.knn_y = np.empty((0,), dtype=np.int8)
        self.candidate_poses = []
        self.pose_scores = {}

        latched_qos = QoSProfile(depth=1)
        latched_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        latched_qos.reliability = ReliabilityPolicy.RELIABLE

        self.pose_pub = self.create_publisher(PoseStamped, self.POSE_TOPIC, latched_qos)
        self.pose_cov_pub = self.create_publisher(PoseWithCovarianceStamped, self.POSE_COV_TOPIC, latched_qos)
        self.initial_pose_pub = self.create_publisher(PoseWithCovarianceStamped, self.INITIAL_POSE_TOPIC, 10)

        self.map_client = self.create_client(GetMap, self.MAP_SERVICE)
        self.scan_topic = self.SCAN_TOPIC
        self.map_timer = self.create_timer(0.5, self.try_request_map)
        self.get_logger().info('localisation_node alive')

    def try_request_map(self):
        if self.map_loaded:
            return

        wait_timeout = min(0.5, self.MAP_SERVICE_TIMEOUT_SEC)
        if not self.map_client.wait_for_service(timeout_sec=wait_timeout):
            self.get_logger().info('waiting for /map_server/map service...')
            return

        if self.map_request_future is None:
            self.get_logger().info('requesting map from /map_server/map service')
            request = GetMap.Request()
            self.map_request_future = self.map_client.call_async(request)
            self.map_request_future.add_done_callback(self.map_response_callback)

    def map_response_callback(self, future):
        self.map_request_future = None

        try:
            response = future.result()
        except Exception as exc:
            self.get_logger().error(f'GetMap request failed: {exc}')
            return

        if response is None:
            self.get_logger().error('GetMap returned no response')
            return

        self.grid_map = GridMap.from_msg(response.map, occupied_threshold=self.OCCUPIED_THRESHOLD)
        self.map_array = self.occupancy_grid_to_numpy(response.map)
        self.free_positions, self.wall_positions = self.extract_map_positions(response.map)
        self.knn_x, self.knn_y = self.build_knn_dataset(self.free_positions, self.wall_positions)
        self.candidate_poses = self.generate_candidate_integer_poses()
        self.map_loaded = True
        self.map_timer.cancel()

        neighbor_count = max(1, self.KNN_NEIGHBORS)
        self.knn_model = KNeighborsClassifier(n_neighbors=neighbor_count)
        self.knn_model.fit(self.knn_x, self.knn_y)

        summary = self.grid_map.summary()

        self.get_logger().info(
            'map received: '
            f"width={summary['width']} height={summary['height']} "
            f"resolution={summary['resolution']:.6f} "
            f"origin=({summary['origin_x']:.3f}, {summary['origin_y']:.3f})"
        )
        self.get_logger().info(
            'map cells: '
            f"free={summary['free_cells']} occupied={summary['occupied_cells']} unknown={summary['unknown_cells']}"
        )
        self.get_logger().info(
            'knn dataset prepared: '
            f'free_positions={len(self.free_positions)} wall_positions={len(self.wall_positions)} '
            f'samples={len(self.knn_x)} neighbors={neighbor_count}'
        )
        self.get_logger().info(
            'candidate poses prepared: '
            f'count={len(self.candidate_poses)} integer_yaw_assumption=0.0'
        )

        sample_world = (0.0, 0.0)
        sample_grid = self.grid_map.world_to_grid(*sample_world)
        roundtrip_world = self.grid_map.grid_to_world(*sample_grid)
        self.get_logger().info(
            'map conversion check: '
            f'world{sample_world} -> grid{sample_grid} -> world{roundtrip_world}'
        )
        self.get_logger().info(
            'map occupancy check: '
            f'in_bounds={self.grid_map.in_bounds(*sample_grid)} '
            f'free={self.grid_map.is_free(*sample_grid)} '
            f'occupied={self.grid_map.is_occupied(*sample_grid)} '
            f'unknown={self.grid_map.is_unknown(*sample_grid)}'
        )

        if self.scan_subscription is None:
            self.scan_subscription = self.create_subscription(LaserScan, self.scan_topic, self.scan_callback, 10)
            self.get_logger().info('scan subscription created; waiting for first scan')

    def scan_callback(self, msg):
        if self.grid_map is None or self.knn_model is None or self.received_scan_once:
            return

        points = self.scan_to_cartesian(msg)
        valid_ranges = len(points)
        self.received_scan_once = True

        self.get_logger().info(
            'scan received: '
            f'frame={msg.header.frame_id} total_ranges={len(msg.ranges)} valid_points={valid_ranges} '
            f'range_min={msg.range_min:.3f} range_max={msg.range_max:.3f}'
        )

        if points.size == 0:
            self.received_scan_once = False
            self.get_logger().warning('first scan produced no valid cartesian points')
            return

        first_point = points[0]
        self.get_logger().info(
            'scan cartesian check: '
            f'first_point=({first_point[0]:.3f}, {first_point[1]:.3f}) sample_count={len(points)}'
        )

        best_pose, best_score = self.estimate_initial_pose(points)
        if best_pose is None:
            self.get_logger().warning('failed to estimate initial pose from first scan')
            return

        self.publish_estimated_pose(msg, best_pose)
        self.get_logger().info(
            'initial localisation result: '
            f'x={best_pose[0]:.3f} y={best_pose[1]:.3f} yaw=0.000 score={best_score:.4f}'
        )

        if self.scan_subscription is not None:
            self.destroy_subscription(self.scan_subscription)
            self.scan_subscription = None
            self.get_logger().info('first scan processed; scan subscription removed')

    def occupancy_grid_to_numpy(self, map_msg: OccupancyGrid):
        return np.array(map_msg.data, dtype=np.int16).reshape((map_msg.info.height, map_msg.info.width))

    def extract_map_positions(self, map_msg: OccupancyGrid):
        free_positions = []
        wall_positions = []

        for gy in range(map_msg.info.height):
            for gx in range(map_msg.info.width):
                value = self.map_array[gy, gx]
                world_x, world_y = self.grid_map.grid_to_world(gx, gy)
                if 0 <= value < self.grid_map.occupied_threshold:
                    free_positions.append((world_x, world_y))
                elif value >= self.grid_map.occupied_threshold:
                    wall_positions.append((world_x, world_y))

        return (
            np.array(free_positions, dtype=np.float32),
            np.array(wall_positions, dtype=np.float32),
        )

    def build_knn_dataset(self, free_positions, wall_positions):
        # Training set for the kNN classifier: X = world coordinates of every map
        # cell, y = its label (0 = free, 1 = wall). With k=1 this turns the discrete
        # map into a continuous "is this point on a wall?" classifier.
        x = np.vstack([free_positions, wall_positions]).astype(np.float32)
        y = np.concatenate([
            np.zeros(len(free_positions), dtype=np.int8),
            np.ones(len(wall_positions), dtype=np.int8),
        ])
        return x, y

    def scan_to_cartesian(self, msg):
        # Convert the LaserScan from polar (range at each beam angle) to cartesian
        # (x, y) points in the robot frame. We take every `downsample`-th beam for
        # speed and drop invalid readings (inf/NaN or outside [range_min, range_max]).
        downsample = max(1, self.SCAN_DOWNSAMPLE_STEP)
        ranges = np.asarray(msg.ranges, dtype=np.float32)
        indices = np.arange(0, len(ranges), downsample, dtype=np.int32)
        sampled_ranges = ranges[indices]
        sampled_angles = msg.angle_min + indices.astype(np.float32) * msg.angle_increment

        valid_mask = (
            np.isfinite(sampled_ranges)
            & (sampled_ranges >= msg.range_min)
            & (sampled_ranges <= msg.range_max)
        )
        if not np.any(valid_mask):
            return np.empty((0, 2), dtype=np.float32)

        valid_ranges = sampled_ranges[valid_mask]
        valid_angles = sampled_angles[valid_mask]

        # Shift scan hits outward by half a cell so they line up better with wall cells.
        adjusted_ranges = valid_ranges + (0.5 * self.grid_map.resolution)
        x_coords = adjusted_ranges * np.cos(valid_angles)
        y_coords = adjusted_ranges * np.sin(valid_angles)
        return np.column_stack((x_coords, y_coords)).astype(np.float32)

    def estimate_initial_pose(self, scan_points):
        # Try every candidate integer pose and keep the highest-scoring one. The
        # winner is our estimate of where the robot actually is.
        best_pose = None
        best_score = float('-inf')
        self.pose_scores = {}

        for pose in self.candidate_poses:
            score = self.score_pose(pose, scan_points)
            self.pose_scores[(pose[0], pose[1])] = score
            if score > best_score:
                best_score = score
                best_pose = pose

        return best_pose, best_score

    def score_pose(self, pose, scan_points):
        # Score a candidate pose: if the robot really were here, the laser hits
        # should fall on walls. We only consider yaw=0 (cookbook assumption) and
        # free cells, then translate the scan to this pose and ask the kNN how many
        # hits land on a wall. Score = fraction of hits classified as wall
        # (1.0 = perfect match).
        x, y, yaw = pose
        if abs(yaw) > 1e-9:
            return float('-inf')

        gx, gy = self.grid_map.world_to_grid(x, y)
        if not self.grid_map.is_free(gx, gy):
            return float('-inf')

        translated_scan = scan_points + np.array([x, y], dtype=np.float32)
        predictions = self.knn_model.predict(translated_scan)
        wall_matches = int(np.count_nonzero(predictions == 1))
        return wall_matches / float(len(predictions))

    def generate_candidate_integer_poses(self):
        # Build the list of poses the robot could be at. By cookbook assumption the
        # robot starts on integer world coordinates facing +x (yaw=0), so we only
        # enumerate the integer (x, y) inside the map that fall on a free cell.
        max_x = self.grid_map.origin_x + self.grid_map.width * self.grid_map.resolution - 1e-9
        max_y = self.grid_map.origin_y + self.grid_map.height * self.grid_map.resolution - 1e-9

        min_ix = int(math.ceil(self.grid_map.origin_x))
        max_ix = int(math.floor(max_x))
        min_iy = int(math.ceil(self.grid_map.origin_y))
        max_iy = int(math.floor(max_y))

        poses = []
        for x in range(min_ix, max_ix + 1):
            for y in range(min_iy, max_iy + 1):
                gx, gy = self.grid_map.world_to_grid(float(x), float(y))
                if self.grid_map.is_free(gx, gy):
                    poses.append((float(x), float(y), 0.0))
        return poses

    def publish_estimated_pose(self, scan_msg, pose):
        x, y, yaw = pose
        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)

        pose_msg = PoseStamped()
        pose_msg.header.stamp = scan_msg.header.stamp
        pose_msg.header.frame_id = 'map'
        pose_msg.pose.position.x = x
        pose_msg.pose.position.y = y
        pose_msg.pose.orientation.z = qz
        pose_msg.pose.orientation.w = qw
        self.pose_pub.publish(pose_msg)

        cov_msg = PoseWithCovarianceStamped()
        cov_msg.header = pose_msg.header
        cov_msg.pose.pose = pose_msg.pose
        cov_msg.pose.covariance[0] = 0.05
        cov_msg.pose.covariance[7] = 0.05
        cov_msg.pose.covariance[35] = 0.1
        self.pose_cov_pub.publish(cov_msg)
        self.initial_pose_pub.publish(cov_msg)


def main(args=None):
    rclpy.init(args=args)
    node = LocalisationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
