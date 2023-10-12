# Joseph Sommer and Yash Mhaskar

from __future__ import print_function
import cv2 as cv
import argparse
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy
import sys
import numpy as np
from cv_bridge import CvBridge

class ObjectRangePubsub(Node):
	def __init__(self):
		super().__init__('object_range_pub_sub')

		#direction subscriber from detect_object

		#Set up QoS Profiles for passing images over WiFi
		image_qos_profile = QoSProfile(
			reliability=QoSReliabilityPolicy.BEST_EFFORT,
			history=QoSHistoryPolicy.KEEP_LAST,
			durability=QoSDurabilityPolicy.VOLATILE,
			depth=1
		)

		#Declare get_direction node is subcribing to the /direction topic.
		self.get_direction = self.create_subscription(
			Int32,
			'direction',
			self._dir_callback,
			image_qos_profile)
		self.get_direction # Prevents unused variable warning.
		
		#Declare get_lidar node is subcribing to the /scan topic.
		self.get_lidar = self.create_subscription(
			Laserscan,
			'scan',
			self._lidar_callback,
			image_qos_profile)
		self.get_direction # Prevents unused variable warning.

		#Declare dir_publisher
		self.movement_publisher = self.create_publisher(Int32, 'movement_coord', 10)
		#self.timer = self.create_timer(0.5, self.publish_command)
	
	global x
	global radius
	global got_dir
	global got_lidar

	def _dir_callback(self, msg):
		x = float(msg.data1)
		radius = float(msg.data2)
		got_dir = 1
		self.pub_coord()

	def _lidar_callback(self,msg):
		range = float(msg.ranges)
		got_lidar = 1
		self.pub_coord()

	def pub_coord():
		if got_dir == 1 and got_lidar == 1:
			#logic
			# publish direction
			msg = Int32()
			msg.data1 = int(x_d)
			msg.data2 = rad
			# Publish the x-axis position
			self.movement_publisher.publish(msg)
			got_dir = 0
			got_lidar = 0

def main(args=None):
	# Setting up publisher values
	rclpy.init(args=args)
	object_range_pub_sub=ObjectRangePubsub()
	rclpy.spin(object_range_pub_sub)

	rclpy.shutdown()

if __name__=='__main__':
  main()
