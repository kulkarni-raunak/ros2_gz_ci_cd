import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String


class MyPyNode(Node):

    def __init__(self):
        super().__init__('my_py_node')
        self.name_msg = String()
        self.name_msg.data = self.get_name()

        self.publisher_ = self.create_publisher(String, '/my_name', 10)
        self.subscription = self.create_subscription(
            Bool,
            '/share_node_name',
            self.listener_callback,
            10)

    def listener_callback(self, msg):
        if msg.data:
            self.get_logger().info(
                f'Received True, publishing my name: {self.get_name()}')
            self.publisher_.publish(self.name_msg)


def main(args=None):
    rclpy.init(args=args)
    node = MyPyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
