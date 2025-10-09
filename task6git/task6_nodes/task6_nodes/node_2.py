import rclpy
from rclpy.node import Node
from task6_interface.msg import RT2String
from std_msgs.msg import String, Int32


class Node2PubSub(Node):
    def __init__(self):
        super().__init__('tasktwopubsub')

        # Subscribes to Node1
        self.subscription12 = self.create_subscription(
            RT2String, 'topic1', self.callbackn1, 10
        )

        # Publishes to Node3
        self.publisher23 = self.create_publisher(Int32, 'topic2', 10)

        # Subscribes to Node3 
        self.subscription32 = self.create_subscription(
            String, 'topic3', self.callbackn2, 10
        )

        self.is_palindrome = 0

    def callbackn1(self, msg: RT2String):
        ptest = msg.paltask6
        self.get_logger().info(f"Received '{ptest}' from node1")

        if ptest == ptest[::-1]:
            self.is_palindrome = 1
        else:
            self.is_palindrome = 0

        result_msg = Int32()
        result_msg.data = self.is_palindrome
        self.publisher23.publish(result_msg)

        self.get_logger().info(f"Published {self.is_palindrome} to node3")

    def callbackn2(self, msg: String):
        self.get_logger().info(f"Received '{msg.data}' from node3")

def main(args=None):
    rclpy.init(args=args)
    node = Node2PubSub()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt, shutting down.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
