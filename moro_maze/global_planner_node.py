import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from moro_maze.map_utils import GridMap
from moro_maze.search_utils import (
    bfs_search,
    build_graph,
    expand_path_cells,
    path_name_cost,
    path_names_to_cells,
    reconstruct_path,
    to_node_name,
)


class GlobalPlannerNode(Node):
    def __init__(self):
        super().__init__('global_planner_node')
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('pose_topic', '/estimated_pose')
        self.declare_parameter('path_topic', '/global_path')
        self.declare_parameter('occupied_threshold', 65)
        self.declare_parameter('minimum_exit_run', 2)
        self.declare_parameter('graph_step', 6)

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
        # Receive the static maze map once and convert it into a helper grid structure
        # that the planner can query in grid coordinates.
        if self.grid_map is None:
            threshold = int(self.get_parameter('occupied_threshold').value)
            self.grid_map = GridMap.from_msg(msg, occupied_threshold=threshold)
            self.get_logger().info(
                f'planner map received: width={self.grid_map.width} height={self.grid_map.height} '
                f'resolution={self.grid_map.resolution:.6f}'
            )
            self.try_plan_test_path()

    def pose_callback(self, msg):
        # Store the current estimated robot position in world coordinates.
        # As soon as both map and pose exist, planning can start.
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
            self.get_logger().warning(f'graph search failed before build: start={start} exits={exits}')
            self.has_logged_plan = True
            return

        graph_step = max(1, int(self.get_parameter('graph_step').value))
        exit_goal_cells = [self.interior_goal_cell(exit_cell, graph_step) for exit_cell in exits]
        exit_goal_cells = [goal_cell for goal_cell in exit_goal_cells if goal_cell is not None]

        required_cells = {start, *exit_goal_cells}
        graph_nodes, graph_edges = build_graph(
            self.grid_map,
            required_cells=required_cells,
            graph_step=graph_step,
        )
        start_name = to_node_name(*start)
        goal_cells = list(dict.fromkeys(exit_goal_cells))
        goal_names = [to_node_name(*goal_cell) for goal_cell in goal_cells]

        self.get_logger().info(
            f'graph built: nodes={len(graph_nodes)} edges={len(graph_edges)} '
            f'start={start_name} graph_step={graph_step}'
        )
        if graph_edges:
            self.get_logger().info(f'graph sample edges: {graph_edges[:10]}')

        best_goal_name = None
        best_path_names = []
        best_discovered = []

        for goal_name in goal_names:
            discovered = bfs_search(start_name, graph_edges, goal_name)
            path_names = reconstruct_path(discovered, start_name, goal_name)
            if path_names and (
                not best_path_names or path_name_cost(path_names) < path_name_cost(best_path_names)
            ):
                best_goal_name = goal_name
                best_path_names = path_names
                best_discovered = discovered

        if not best_path_names:
            self.get_logger().warning(
                f'graph search failed: start={start_name} exits={goal_names} edges={len(graph_edges)}'
            )
            self.has_logged_plan = True
            return

        self.has_logged_plan = True
        path_cells = path_names_to_cells(best_path_names)
        expanded_path_cells = expand_path_cells(path_cells)

        self.get_logger().info(
            f'bfs exit success: start={start_name} goal={best_goal_name} '
            f'visited_edges={len(best_discovered)} path_nodes={len(best_path_names)}'
        )
        self.get_logger().info(
            f'bfs found path: {best_path_names}'
        )
        self.get_logger().info(
            f'expanded path cells: graph_nodes={len(path_cells)} path_waypoints={len(expanded_path_cells)}'
        )
        self.get_logger().info(
            f'graph first-last cells: first={expanded_path_cells[0]} last={expanded_path_cells[-1]}'
        )
        self.publish_path(expanded_path_cells)

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

    def interior_goal_cell(self, exit_cell, graph_step):
        gx, gy = exit_cell
        step = max(1, int(graph_step))

        if gx == 0:
            candidate = (min(self.grid_map.width - 1, gx + step), gy)
        elif gx == self.grid_map.width - 1:
            candidate = (max(0, gx - step), gy)
        elif gy == 0:
            candidate = (gx, min(self.grid_map.height - 1, gy + step))
        elif gy == self.grid_map.height - 1:
            candidate = (gx, max(0, gy - step))
        else:
            candidate = (gx, gy)

        return self.grid_map.nearest_free_cell(*candidate, max_radius=step)


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
