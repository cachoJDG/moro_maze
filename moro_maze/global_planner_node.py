import math

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from moro_maze.map_utils import GridMap
from moro_maze.search_utils import (
    bfs_search,
    build_graph,
    expand_sparse_path,
    line_is_free,
    nearest_visible_node,
    path_name_cost,
    path_names_to_cells,
    reconstruct_path,
    to_node_name,
)


class GlobalPlannerNode(Node):
    # --- Tuning constants (hardcoded; kept simple, no ROS parameters) -------
    MAP_TOPIC = '/map'
    POSE_TOPIC = '/estimated_pose'
    PATH_TOPIC = '/global_path'
    OCCUPIED_THRESHOLD = 65      # occupancy value at/above which a cell is a wall
    MINIMUM_EXIT_RUN = 2         # min free cells in a wall side to count as an exit
    GRAPH_STEP = 6               # visibility-graph lattice spacing (cells)
    EXECUTION_STEP_CELLS = 2     # densify the sparse path every N cells
    # How far inside the border the final doorway waypoint sits. The MORO local
    # planner drives /cmd_vel directly (Nav2 is NOT used for control), so the old
    # reason for a big inset -- keeping Nav2's NavfnPlanner from propagating off
    # the costmap edge -- no longer applies. 1 cell puts the last waypoint just
    # past the wall line, so the robot drives OUT through the gap instead of
    # stopping short inside the maze.
    EXIT_INSET_CELLS = 1

    def __init__(self):
        super().__init__('global_planner_node')

        self.grid_map = None
        self.robot_pose = None
        self.has_logged_plan = False

        map_qos = QoSProfile(depth=1)
        map_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        map_qos.reliability = ReliabilityPolicy.RELIABLE

        path_qos = QoSProfile(depth=1)
        path_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        path_qos.reliability = ReliabilityPolicy.RELIABLE

        self.path_pub = self.create_publisher(Path, self.PATH_TOPIC, path_qos)

        self.create_subscription(OccupancyGrid, self.MAP_TOPIC, self.map_callback, map_qos)
        self.create_subscription(PoseStamped, self.POSE_TOPIC, self.pose_callback, 10)
        self.get_logger().info('global_planner_node alive')

    def map_callback(self, msg):
        # Receive the static maze map once and convert it into a helper grid structure
        # that the planner can query in grid coordinates.
        if self.grid_map is None:
            self.grid_map = GridMap.from_msg(msg, occupied_threshold=self.OCCUPIED_THRESHOLD)
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
        exits = self.grid_map.detect_border_exits(minimum_run=self.MINIMUM_EXIT_RUN)
        self.get_logger().info(f'detected exits: {exits}')

        if start is None or not exits:
            self.get_logger().warning(f'graph search failed before build: start={start} exits={exits}')
            self.has_logged_plan = True
            return

        requested_graph_step = max(1, self.GRAPH_STEP)
        attempted_steps = []
        fallback_steps = [requested_graph_step]
        if requested_graph_step > 3:
            fallback_steps.append(3)
        fallback_steps.append(1)

        graph_step = None
        graph_nodes = []
        graph_edges = []
        start_name = None
        best_goal_name = None
        best_path_names = []
        best_discovered = []
        final_exit_goal_cells = []

        for candidate_step in dict.fromkeys(fallback_steps):
            attempted_steps.append(candidate_step)
            self.get_logger().info(f'graph attempt: graph_step={candidate_step}')
            exit_goal_cells = [self.interior_goal_cell(exit_cell, candidate_step) for exit_cell in exits]
            exit_goal_cells = [goal_cell for goal_cell in exit_goal_cells if goal_cell is not None]

            graph_nodes, graph_edges = build_graph(
                self.grid_map,
                required_cells=None,
                graph_step=candidate_step,
            )
            start_anchor = nearest_visible_node(start, graph_nodes, self.grid_map)
            goal_cells = []
            for exit_goal_cell in exit_goal_cells:
                anchor = nearest_visible_node(exit_goal_cell, graph_nodes, self.grid_map)
                if anchor is not None:
                    goal_cells.append(anchor)

            self.get_logger().info(
                f'graph anchors: graph_step={candidate_step} start_cell={start} start_anchor={start_anchor} '
                f'exit_goals={exit_goal_cells} anchored_goals={goal_cells}'
            )

            goal_cells = list(dict.fromkeys(goal_cells))
            if start_anchor is None or not goal_cells:
                self.get_logger().warning(
                    f'graph anchoring failed: graph_step={candidate_step} '
                    f'start_anchor={start_anchor} goal_cells={goal_cells}'
                )
                continue

            start_name = to_node_name(*start_anchor)
            goal_names = [to_node_name(*goal_cell) for goal_cell in goal_cells]

            self.get_logger().info(
                f'graph built: nodes={len(graph_nodes)} edges={len(graph_edges)} '
                f'start={start_name} graph_step={candidate_step}'
            )
            if graph_edges:
                self.get_logger().info(f'graph sample edges: {graph_edges[:10]}')

            found_for_step = False
            for goal_name in goal_names:
                discovered = bfs_search(start_name, graph_edges, goal_name)
                path_names = reconstruct_path(discovered, start_name, goal_name)
                if path_names and (
                    not best_path_names or path_name_cost(path_names) < path_name_cost(best_path_names)
                ):
                    graph_step = candidate_step
                    best_goal_name = goal_name
                    best_path_names = path_names
                    best_discovered = discovered
                    final_exit_goal_cells = exit_goal_cells
                    found_for_step = True

            if found_for_step:
                self.get_logger().info(f'graph attempt succeeded: graph_step={candidate_step}')
                break
            self.get_logger().warning(f'graph attempt found no path: graph_step={candidate_step}')

        if not best_path_names:
            self.get_logger().warning(
                f'graph search failed after steps={attempted_steps} start={start} exits={exits}'
            )
            self.has_logged_plan = True
            return

        self.has_logged_plan = True
        path_cells = path_names_to_cells(best_path_names)
        if start != path_cells[0] and line_is_free(self.grid_map, start, path_cells[0]):
            path_cells.insert(0, start)
        # Overshoot correction only. If the last graph node overshoots the real
        # exit (the interior exit goal is closer to the PREVIOUS waypoint than the
        # node is), heading to the node and then back forces a doubling-back turn,
        # so replace the overshooting node with the exit goal when there is line of
        # sight. We deliberately do NOT *append* the interior exit goal: it sits a
        # full graph step (~6 cells) inside the opening, so appending it after a
        # node that is already closer to the border sends the robot backward into
        # the maze (a visible left/west detour). The doorway extension below is
        # what drives the robot OUT, toward the actual border gap.
        if final_exit_goal_cells and len(path_cells) >= 2:
            final_exit_goal = min(
                final_exit_goal_cells,
                key=lambda cell: abs(cell[0] - path_cells[-1][0]) + abs(cell[1] - path_cells[-1][1]),
            )
            prev_cell = path_cells[-2]
            last_cell = path_cells[-1]
            if final_exit_goal != last_cell:
                goal_dist = math.hypot(final_exit_goal[0] - prev_cell[0], final_exit_goal[1] - prev_cell[1])
                node_dist = math.hypot(last_cell[0] - prev_cell[0], last_cell[1] - prev_cell[1])
                if goal_dist < node_dist and line_is_free(self.grid_map, prev_cell, final_exit_goal):
                    self.get_logger().info(
                        f'dropping overshoot node {last_cell} -> exit {final_exit_goal} '
                        f'(goal_dist={goal_dist:.2f} < node_dist={node_dist:.2f})'
                    )
                    path_cells[-1] = final_exit_goal

        # Drive out through the doorway. The BFS goal sits a full graph step inside the
        # maze (so it connects cleanly to the graph), which leaves the robot stopping
        # ~1 m short of the border. Extend the path toward the actual border exit, a
        # couple of cells inside the edge, so the robot visibly escapes. Only do this
        # when there is clear line of sight; otherwise keep the safe interior goal.
        if exits:
            exit_inset = max(1, self.EXIT_INSET_CELLS)
            nearest_exit = min(
                exits,
                key=lambda cell: abs(cell[0] - path_cells[-1][0]) + abs(cell[1] - path_cells[-1][1]),
            )
            escape_cell = self.interior_goal_cell(nearest_exit, exit_inset)
            if (
                escape_cell is not None
                and escape_cell != path_cells[-1]
                and line_is_free(self.grid_map, path_cells[-1], escape_cell)
            ):
                self.get_logger().info(
                    f'extending to doorway: exit={nearest_exit} escape_cell={escape_cell} inset={exit_inset}'
                )
                path_cells.append(escape_cell)

        self.get_logger().info(
            f'bfs exit success: start={start_name} goal={best_goal_name} '
            f'visited_edges={len(best_discovered)} path_nodes={len(best_path_names)}'
        )
        self.get_logger().info(
            f'bfs found path: {best_path_names}'
        )
        self.get_logger().info(
            f'graph path cells: waypoints={len(path_cells)} graph_step={graph_step}'
        )
        self.get_logger().info(
            f'graph first-last cells: first={path_cells[0]} last={path_cells[-1]}'
        )

        execution_step = max(1, self.EXECUTION_STEP_CELLS)
        dense_path_cells = expand_sparse_path(self.grid_map, path_cells, step_cells=execution_step)
        self.get_logger().info(
            f'path densified: sparse_waypoints={len(path_cells)} '
            f'dense_waypoints={len(dense_path_cells)} execution_step_cells={execution_step}'
        )
        self.publish_path(dense_path_cells)

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
