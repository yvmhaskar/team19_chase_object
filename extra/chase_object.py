# Joseph Sommer and Yash Mhaskar

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from geometry_msgs.msg import Twist
import numpy

class MovementSubPub(Node):
	def __init__(self):
		super().__init__('movement_sub_pub')
		self.subscription = self.create_subscription(Int32, 'movement_coord', self.mover_callback, 5)
		self.subscription
		self.cmd_vel = self.create_publisher(Twist,'cmd_vel',10)

	def mover_callback(self, msg):
		avg_speed = (numpy.pi)/4
		turn_dir = float(msg.data)
		twist = Twist()
		twist.angular.z = avg_speed*turn_dir
		self.cmd_vel.publish(twist)

def main(args=None):
	rclpy.init(args=args)
	# Subscriber
	rotate_sub_pub = RotateSubPub()
	rclpy.spin(rotate_sub_pub)

	rotate_sub_pub.destroy_node()


	# publisher
#    vel_publisher=VelPublisher()
#    rclpy.spin(vel_publisher)
#    vel_publisher.destroy_node()

	rclpy.shutdown()

if __name__ == '__main__':
	main()
