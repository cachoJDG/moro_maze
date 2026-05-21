import rclpy
from rclpy.node import Node


class GlobalPlannerNode(Node):
    def __init__(self):
        super().__init__('global_planner_node')
        self.get_logger().info('global_planner_node alive')


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
