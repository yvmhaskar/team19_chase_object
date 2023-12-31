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
		e_a = e_a*3.14159/180
		e_a = numpy.arctan2(numpy.sin(e_a), numpy.cos(e_a))

		setlin = 0.5
		setang = 0.0
		bufferlin = 0.1
		bufferang = 0.2
		#if (abs(e_l)-setlin>bufferlin):
		#	pid_l = PID(-0.3,0.0,0.001,setpoint=setlin)
		#	v_l = pid_l(e_l)
		#else:
		#	v_l = 0.0
		#if (abs(e_a)-setang>bufferang):
		#	pid_a = PID(-0.65,0.0,0.002,setpoint=setang)
		#	v_a = pid_a(e_a)
		#else:
		#	v_a = 0.0
		

		pid_l = PID(-0.45,0.000,0.05,setpoint=setlin)
		v_l = pid_l(e_l)
		pid_a = PID(-0.6,0.0,0.05,setpoint=setang)
		v_a = pid_a(e_a)
		#v_l = -0.45*e_l
		#v_a = 0.2*e_a	
		#v_l = pid_l(e_l)
		#v_a = pid_a(e_a)
		if v_l > 0.22:
			v_l = 0.22
		elif v_l < -0.22:
			v_l = -0.22
		if v_a > 1.42:
			v_a = 1.42
		elif v_a < -1.42:
			v_a = -1.42
		
		twist = Twist()
		if abs(e_a) > 0.2:
			self.get_logger().info('e_a: "%s"'% e_a)
			self.get_logger().info('v_a: "%s"'% v_a)
			twist.angular.z = v_a
			twist.linear.x = 0.0
			case = 1
			
		if abs(e_a) < 0.2:
			self.get_logger().info('e_l: "%s"'% e_l)
			self.get_logger().info('v_l: "%s"'% v_l)
			twist.angular.z = 0.0
			twist.linear.x = v_l
			case = 2

		if abs(e_l)<0.55 and abs(e_l)>0.45:
			self.get_logger().info('e_l: "%s"'% e_l)
			self.get_logger().info('v_l: "%s"'% v_l)
			twist.angular.z = 0.0
			twist.linear.x = 0.0
			
		
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
