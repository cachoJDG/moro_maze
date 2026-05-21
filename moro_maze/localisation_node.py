import math

import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan

from moro_maze.map_utils import GridMap


class LocalisationNode(Node):
    def __init__(self):
        super().__init__('localisation_node')
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('pose_topic', '/estimated_pose')
        self.declare_parameter('pose_cov_topic', '/estimated_pose_cov')
        self.declare_parameter('occupied_threshold', 65)
        self.declare_parameter('scan_downsample_step', 8)
        self.declare_parameter('spawn_min', 0)
        self.declare_parameter('spawn_max', 3)
        self.declare_parameter('theta_samples', 16)
        self.declare_parameter('refine_xy_step', 0.1)
        self.declare_parameter('refine_xy_radius', 0.2)
        self.declare_parameter('refine_theta_step', 0.2)
        self.declare_parameter('refine_theta_radius', 0.4)
        self.declare_parameter('tracking_log_period_scans', 20)

        self.grid_map = None
        self.received_scan_once = False
        self.initial_pose_done = False
        self.latest_points = []
        self.estimated_pose = None
        self.scan_counter = 0

        map_topic = self.get_parameter('map_topic').value
        scan_topic = self.get_parameter('scan_topic').value
        pose_topic = self.get_parameter('pose_topic').value
        pose_cov_topic = self.get_parameter('pose_cov_topic').value

        latched_qos = QoSProfile(depth=1)
        latched_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        latched_qos.reliability = ReliabilityPolicy.RELIABLE

        self.pose_pub = self.create_publisher(PoseStamped, pose_topic, latched_qos)
        self.pose_cov_pub = self.create_publisher(PoseWithCovarianceStamped, pose_cov_topic, latched_qos)

        self.create_subscription(OccupancyGrid, map_topic, self.map_callback, latched_qos)
        self.create_subscription(LaserScan, scan_topic, self.scan_callback, 10)
        self.get_logger().info('localisation_node alive')

    def map_callback(self, msg):
        if self.grid_map is not None:
            return

        threshold = int(self.get_parameter('occupied_threshold').value)
        self.grid_map = GridMap.from_msg(msg, occupied_threshold=threshold)
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

    def scan_callback(self, msg):
        points = self.scan_to_cartesian(msg)
        self.latest_points = points
        self.scan_counter += 1
        valid_ranges = len(points)

        if not self.received_scan_once:
            self.received_scan_once = True
            self.get_logger().info(
                'scan received: '
                f'frame={msg.header.frame_id} total_ranges={len(msg.ranges)} valid_points={valid_ranges} '
                f'range_min={msg.range_min:.3f} range_max={msg.range_max:.3f}'
            )

            if points:
                first_point = points[0]
                self.get_logger().info(
                    'scan cartesian check: '
                    f'first_point=({first_point[0]:.3f}, {first_point[1]:.3f}) '
                    f'sample_count={len(points)}'
                )

        if self.grid_map is not None and not self.initial_pose_done and points:
            best_pose, best_score = self.estimate_initial_pose(points)
            if best_pose is not None:
                self.initial_pose_done = True
                self.estimated_pose = best_pose
                self.publish_estimated_pose(msg, best_pose)
                self.get_logger().info(
                    'initial localisation result: '
                    f'x={best_pose[0]:.3f} y={best_pose[1]:.3f} yaw={best_pose[2]:.3f} score={best_score:.4f}'
                )
        elif self.grid_map is not None and self.initial_pose_done and self.estimated_pose is not None and points:
            tracked_pose, tracked_score = self.refine_pose(self.estimated_pose, points)
            self.estimated_pose = tracked_pose
            self.publish_estimated_pose(msg, tracked_pose)

            log_period = max(1, int(self.get_parameter('tracking_log_period_scans').value))
            if self.scan_counter % log_period == 0:
                self.get_logger().info(
                    'tracking localisation update: '
                    f'x={tracked_pose[0]:.3f} y={tracked_pose[1]:.3f} yaw={tracked_pose[2]:.3f} score={tracked_score:.4f}'
                )

    def scan_to_cartesian(self, msg):
        downsample = max(1, int(self.get_parameter('scan_downsample_step').value))
        points = []
        angle = msg.angle_min

        for index, distance in enumerate(msg.ranges):
            if index % downsample == 0 and math.isfinite(distance):
                if msg.range_min <= distance <= msg.range_max:
                    x = distance * math.cos(angle)
                    y = distance * math.sin(angle)
                    points.append((x, y))
            angle += msg.angle_increment

        return points

    def estimate_initial_pose(self, scan_points):
        spawn_min = int(self.get_parameter('spawn_min').value)
        spawn_max = int(self.get_parameter('spawn_max').value)
        theta_samples = max(8, int(self.get_parameter('theta_samples').value))

        best_pose = None
        best_score = float('inf')

        for x in range(spawn_min, spawn_max + 1):
            for y in range(spawn_min, spawn_max + 1):
                for index in range(theta_samples):
                    theta = -math.pi + (2.0 * math.pi * index / theta_samples)
                    pose = (float(x), float(y), theta)
                    score = self.score_pose(pose, scan_points)
                    if score < best_score:
                        best_score = score
                        best_pose = pose

        if best_pose is None:
            return None, float('inf')

        refined_pose, refined_score = self.refine_pose(best_pose, scan_points)
        return refined_pose, refined_score

    def refine_pose(self, seed_pose, scan_points):
        xy_step = float(self.get_parameter('refine_xy_step').value)
        xy_radius = float(self.get_parameter('refine_xy_radius').value)
        theta_step = float(self.get_parameter('refine_theta_step').value)
        theta_radius = float(self.get_parameter('refine_theta_radius').value)

        best_pose = seed_pose
        best_score = self.score_pose(seed_pose, scan_points)

        x_values = self.arange_inclusive(seed_pose[0] - xy_radius, seed_pose[0] + xy_radius, xy_step)
        y_values = self.arange_inclusive(seed_pose[1] - xy_radius, seed_pose[1] + xy_radius, xy_step)
        theta_values = self.arange_inclusive(seed_pose[2] - theta_radius, seed_pose[2] + theta_radius, theta_step)

        for x in x_values:
            for y in y_values:
                for theta in theta_values:
                    pose = (float(x), float(y), self.normalize_angle(float(theta)))
                    score = self.score_pose(pose, scan_points)
                    if score < best_score:
                        best_score = score
                        best_pose = pose

        return best_pose, best_score

    def score_pose(self, pose, scan_points):
        x, y, theta = pose
        cos_theta = math.cos(theta)
        sin_theta = math.sin(theta)

        if not self.grid_map.is_free(*self.grid_map.world_to_grid(x, y)):
            return float('inf')

        total_distance = 0.0
        used_points = 0
        outside_penalty = 0.0

        for px, py in scan_points:
            wx = x + cos_theta * px - sin_theta * py
            wy = y + sin_theta * px + cos_theta * py
            gx, gy = self.grid_map.world_to_grid(wx, wy)

            if not self.grid_map.in_bounds(gx, gy):
                outside_penalty += 1.0
                continue

            total_distance += self.grid_map.nearest_obstacle_distance(wx, wy)
            used_points += 1

        if used_points == 0:
            return float('inf')

        return (total_distance / used_points) + 0.2 * outside_penalty

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

    @staticmethod
    def normalize_angle(angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    @staticmethod
    def arange_inclusive(start, stop, step):
        values = []
        current = start
        while current <= stop + 1e-9:
            values.append(current)
            current += step
        return values


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
