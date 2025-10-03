#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from task5_interface.msg import TaskOne

class TaskOneSubscriber(Node):
	def __init__(self):
		super().__init__('taskone_subscriber')
		self.subscription_=self.create_subscription(TaskOne,'/taskone',self.listener_callback,10)
	def listener_callback(self,msg:TaskOne):
		ang_vel=msg.ang_vel
		radius=msg.radius
		longitudnal_speed=ang_vel*radius
		self.get_logger().info(f"received radius {radius} and angular velocity {ang_vel}, longitudnal speed is {longitudnal_speed}")
def main(args=None):
	rclpy.init(args=args)
	node=TaskOneSubscriber()
	rclpy.spin(node)
	node.destroy_node()
	rclpy.shutdown()
if __name__ =='__main__':
	main()
			
