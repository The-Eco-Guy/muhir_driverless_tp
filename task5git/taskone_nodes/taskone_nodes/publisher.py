#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from task5_interface.msg import TaskOne

class TaskOnePublisher(Node):
	def __init__(self):
		super().__init__('taskone_publisher')
		self.publisher_=self.create_publisher(TaskOne,'/taskone',10)
def main(args=None):
	rclpy.init(args=args)
	node=TaskOnePublisher()
	node.get_logger().info("Task one publisher, enter Q to exit")
	while rclpy.ok():
		a=input("enter angular velocity: ")
		
		if(a=='Q'):
			node.get_logger().info("exiting publisher")
			break
		b=input("enter radius: ")
		if(b=='Q'):
			node.get_logger().info("exiting publisher")
			break
		ang_vel=float(a)
		radius=float(b)
		msg=TaskOne()
		msg.ang_vel=ang_vel
		msg.radius=radius
		node.publisher_.publish(msg)
		node.get_logger().info(f"published: angular velocity = {ang_vel} , radius is = {radius}")
	node.destroy_node()
	rclpy.shutdown()
			
