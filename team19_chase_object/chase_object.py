# Joseph Sommer and Yash Mhaskar

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist
import numpy
from simple_pid import PID

class MovementSubPub(Node):
	def __init__(self):
		super().__init__('movement_sub_pub')
		self.subscription = self.create_subscription(Float32MultiArray, 'movement_coord', self.mover_callback, 5)
		self.subscription
		self.cmd_vel = self.create_publisher(Twist,'cmd_vel',10)

	def mover_callback(self, msg):
		mover_data = msg.data
		e_l = float(mover_data[0])
		e_a = float(mover_data[1])

		pid_l = PID(1,0.1,0.05,setpoint=0.5)
		pid_a = PID(1,0.1,0.05,setpoint=0)

		v_l = pid_l(e_l)
		v_a = pid_a(e_a)
		if v_l > 0.22:
			v_l = 0.22
		if v_l < -0.22:
			v_l = -0.22
		if v_a > 2.84:
			v_a = 2.84
		if v_l < -2.84:
			v_l = -2.84
		
		twist = Twist()
		twist.linear.x = v_l
		twist.angular.z = v_a
		self.cmd_vel.publish(twist)

def main(args=None):
	rclpy.init(args=args)
	# Subscriber
	movement_sub_pub = MovementSubPub()
	rclpy.spin(movement_sub_pub)

	movement_sub_pub.destroy_node()


	# publisher
#    vel_publisher=VelPublisher()
#    rclpy.spin(vel_publisher)
#    vel_publisher.destroy_node()

	rclpy.shutdown()

if __name__ == '__main__':
	main()