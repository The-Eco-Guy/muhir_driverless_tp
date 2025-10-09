import rclpy
from rclpy.node import Node
from task6_interface.msg import RT2String


class Node1Publisher(Node):
    def __init__(self):
        super().__init__('nodeone')
        self.publisher = self.create_publisher(RT2String, 'topic1', 10)


def main(args=None):
    rclpy.init(args=args)
    node = Node1Publisher()

    node.get_logger().info("Enter string to check if it is a palindrome (Q to exit).")

    try:
        while rclpy.ok():
            a = input("Enter string to be checked: ")

            if a == 'Q':
                node.get_logger().info("Exiting node1")
                break

            msg = RT2String()
            msg.paltask6 = a
            node.publisher.publish(msg)
            node.get_logger().info(f"Sent: '{msg.paltask6}' from node1 to node2")

    except KeyboardInterrupt:
        node.destroy_node()
        node.get_logger().info("Keyboard interrupt, shutting down.")

    rclpy.shutdown()


if __name__ == '__main__':
    main()
