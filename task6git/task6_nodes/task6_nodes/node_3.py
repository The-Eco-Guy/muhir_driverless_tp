import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32


class Node3PubSub(Node):
    def __init__(self):
        super().__init__('taskthreepubsub')
        self.subscription23 = self.create_subscription(
            Int32, 'topic2', self.callbackn3, 10
        )
        self.publisher32 = self.create_publisher(String, 'topic3', 10)

    def callbackn3(self, msg: Int32):
        if msg.data:
            self.get_logger().info(f"Received {msg.data}, publishing 'yes' to node2")
            self.publisher32.publish(String(data="yes"))
        else:
            self.get_logger().info(f"Received {msg.data}, publishing 'no' to node2")
            self.publisher32.publish(String(data="no"))


def main(args=None):
    rclpy.init(args=args)
    node = Node3PubSub()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt, shutting down.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
