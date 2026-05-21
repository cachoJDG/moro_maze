import rclpy
from rclpy.node import Node


class LocalControllerNode(Node):
    def __init__(self):
        super().__init__('local_controller_node')
        self.get_logger().info('local_controller_node alive')


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
