import math

import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Path
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


class LocalControllerNode(Node):
    # --- Tuning constants (hardcoded; kept simple, no ROS parameters) -------
    PATH_TOPIC = '/global_path'
    ESTIMATED_POSE_TOPIC = '/estimated_pose'
    AMCL_POSE_TOPIC = '/amcl_pose'
    GOAL_TOPIC = '/goal_pose'
    NAVIGATE_ACTION = 'navigate_to_pose'
    GOAL_TOLERANCE = 0.20        # [m] waypoint-reached threshold
    ACTION_WAIT_TIMEOUT = 0.5    # [s] wait for the action server
    RETRY_TIMER_PERIOD = 1.0     # [s] resend goal if server unavailable

    def __init__(self):
        super().__init__('local_controller_node')

        self.path_points = []
        self.current_goal_index = None
        self.current_pose = None
        self.goal_in_progress = False

        path_qos = QoSProfile(depth=1)
        path_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        path_qos.reliability = ReliabilityPolicy.RELIABLE

        goal_qos = QoSProfile(depth=1)
        goal_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        goal_qos.reliability = ReliabilityPolicy.RELIABLE

        self.goal_pub = self.create_publisher(PoseStamped, self.GOAL_TOPIC, goal_qos)
        self.create_subscription(Path, self.PATH_TOPIC, self.path_callback, path_qos)
        self.create_subscription(PoseStamped, self.ESTIMATED_POSE_TOPIC, self.estimated_pose_callback, 10)
        self.create_subscription(PoseWithCovarianceStamped, self.AMCL_POSE_TOPIC, self.amcl_pose_callback, 10)

        self.navigate_client = ActionClient(self, NavigateToPose, self.NAVIGATE_ACTION)
        self.retry_timer = self.create_timer(self.RETRY_TIMER_PERIOD, self.retry_current_goal)
        self.get_logger().info('local_controller_node alive')

    def estimated_pose_callback(self, msg):
        self.current_pose = (msg.pose.position.x, msg.pose.position.y)

    def amcl_pose_callback(self, msg):
        self.current_pose = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def path_callback(self, msg):
        self.path_points = [
            (pose.pose.position.x, pose.pose.position.y)
            for pose in msg.poses
        ]
        self.goal_in_progress = False

        self.get_logger().info(f'controller path received: waypoints={len(self.path_points)}')
        if not self.path_points:
            self.current_goal_index = None
            return

        self.current_goal_index = self.first_relevant_index()
        self.try_send_current_goal(reason='new_path')

    def retry_current_goal(self):
        if self.current_goal_index is None or self.goal_in_progress or not self.path_points:
            return
        self.try_send_current_goal(reason='retry')

    def first_relevant_index(self):
        if self.current_pose is None:
            return 0

        tolerance = self.GOAL_TOLERANCE
        for index, point in enumerate(self.path_points):
            if self.distance(self.current_pose, point) > tolerance:
                return index
        return len(self.path_points) - 1

    def try_send_current_goal(self, reason):
        if self.current_goal_index is None or not self.path_points or self.goal_in_progress:
            return

        if not self.navigate_client.wait_for_server(timeout_sec=self.ACTION_WAIT_TIMEOUT):
            self.get_logger().info(f'navigate_to_pose not ready yet; postponing waypoint send ({reason})')
            return

        x, y = self.path_points[self.current_goal_index]

        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'map'
        pose_msg.pose.position.x = float(x)
        pose_msg.pose.position.y = float(y)
        pose_msg.pose.orientation.w = 1.0

        self.goal_pub.publish(pose_msg)

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose_msg

        self.goal_in_progress = True
        self.get_logger().info(
            f'sending waypoint goal: reason={reason} index={self.current_goal_index} '
            f'x={x:.3f} y={y:.3f}'
        )
        send_future = self.navigate_client.send_goal_async(goal_msg)
        send_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.goal_in_progress = False
            self.get_logger().warning('navigate_to_pose goal was rejected')
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.goal_result_callback)

    def goal_result_callback(self, future):
        self.goal_in_progress = False

        try:
            result = future.result()
        except Exception as exc:
            self.get_logger().warning(f'navigate_to_pose result failed: {exc}')
            return

        status = int(result.status)
        if status != 4:
            self.get_logger().warning(f'navigate_to_pose finished with status={status}')
            return

        self.get_logger().info(f'waypoint goal reached: index={self.current_goal_index}')

        if self.current_goal_index is None or self.current_goal_index >= len(self.path_points) - 1:
            self.current_goal_index = None
            self.get_logger().info('controller goal sequence complete')
            return

        self.current_goal_index += 1
        self.try_send_current_goal(reason='advance')

    @staticmethod
    def distance(a, b):
        return math.hypot(a[0] - b[0], a[1] - b[1])


def main(args=None):
    rclpy.init(args=args)
    node = LocalControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
