import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from moro_maze.map_utils import GridMap
from moro_maze.search_utils import astar_search, shortcut_smooth_path


class GlobalPlannerNode(Node):
    def __init__(self):
        super().__init__('global_planner_node')
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('pose_topic', '/estimated_pose')
        self.declare_parameter('path_topic', '/global_path')
        self.declare_parameter('occupied_threshold', 65)
        self.declare_parameter('connectivity', 8)
        self.declare_parameter('minimum_exit_run', 2)
        self.declare_parameter('smooth_path', True)

        self.grid_map = None
        self.robot_pose = None
        self.has_logged_plan = False

        map_topic = self.get_parameter('map_topic').value
        pose_topic = self.get_parameter('pose_topic').value
        path_topic = self.get_parameter('path_topic').value

        map_qos = QoSProfile(depth=1)
        map_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        map_qos.reliability = ReliabilityPolicy.RELIABLE

        path_qos = QoSProfile(depth=1)
        path_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        path_qos.reliability = ReliabilityPolicy.RELIABLE

        self.path_pub = self.create_publisher(Path, path_topic, path_qos)

        self.create_subscription(OccupancyGrid, map_topic, self.map_callback, map_qos)
        self.create_subscription(PoseStamped, pose_topic, self.pose_callback, 10)
        self.get_logger().info('global_planner_node alive')

    def map_callback(self, msg):
        if self.grid_map is None:
            threshold = int(self.get_parameter('occupied_threshold').value)
            self.grid_map = GridMap.from_msg(msg, occupied_threshold=threshold)
            self.get_logger().info(
                f'planner map received: width={self.grid_map.width} height={self.grid_map.height} '
                f'resolution={self.grid_map.resolution:.6f}'
            )
            self.try_plan_test_path()

    def pose_callback(self, msg):
        self.robot_pose = (msg.pose.position.x, msg.pose.position.y)
        self.try_plan_test_path()

    def try_plan_test_path(self):
        if self.grid_map is None or self.robot_pose is None or self.has_logged_plan:
            return

        start = self.grid_map.world_to_grid(self.robot_pose[0], self.robot_pose[1])
        start = self.grid_map.nearest_free_cell(*start)

        exits = self.grid_map.detect_border_exits(
            minimum_run=int(self.get_parameter('minimum_exit_run').value)
        )
        self.get_logger().info(f'detected exits: {exits}')

        if start is None or not exits:
            self.get_logger().warning(f'astar exit test failed: start={start} exits={exits}')
            self.has_logged_plan = True
            return

        connectivity = int(self.get_parameter('connectivity').value)
        best_goal = None
        best_path = []

        for exit_cell in exits:
            goal = self.grid_map.nearest_free_cell(*exit_cell)
            path = astar_search(self.grid_map, start, goal, connectivity=connectivity)
            if path and (not best_path or len(path) < len(best_path)):
                best_goal = goal
                best_path = path

        if not best_path:
            self.get_logger().warning(
                f'astar exit test failed: start={start} exits={exits} connectivity={connectivity}'
            )
            self.has_logged_plan = True
            return

        self.has_logged_plan = True
        raw_length = len(best_path)
        if bool(self.get_parameter('smooth_path').value):
            best_path = shortcut_smooth_path(self.grid_map, best_path)

        self.get_logger().info(
            f'astar exit success: start={start} goal={best_goal} length={len(best_path)} connectivity={connectivity}'
        )
        self.get_logger().info(
            f'path smoothing: enabled={bool(self.get_parameter("smooth_path").value)} '
            f'raw_waypoints={raw_length} smoothed_waypoints={len(best_path)}'
        )
        self.get_logger().info(f'astar first-last cells: first={best_path[0]} last={best_path[-1]}')
        self.publish_path(best_path)

    def publish_path(self, path_cells):
        msg = Path()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'

        for gx, gy in path_cells:
            pose = PoseStamped()
            pose.header = msg.header
            wx, wy = self.grid_map.grid_to_world(gx, gy)
            pose.pose.position.x = float(wx)
            pose.pose.position.y = float(wy)
            pose.pose.orientation.w = 1.0
            msg.poses.append(pose)

        self.path_pub.publish(msg)
        self.get_logger().info(
            f'published global path: waypoints={len(msg.poses)} frame={msg.header.frame_id}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = GlobalPlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
