# Joseph Sommer and Yash Mhaskar

from __future__ import print_function
import cv2 as cv
import argparse
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32MultiArray
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy
import sys
import numpy as np
from cv_bridge import CvBridge
import math

global x1
global r
global lidar_data
global got_dir
got_dir = 0
global got_lidar
got_lidar = 0

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
			Int32MultiArray,
			'direction',
			self._dir_callback,
			image_qos_profile)
		self.get_direction # Prevents unused variable warning.
		
		#Declare get_lidar node is subcribing to the /scan topic.
		self.get_lidar = self.create_subscription(
			LaserScan,
			'scan',
			self._lidar_callback,
			image_qos_profile)
		self.get_direction # Prevents unused variable warning.

		#Declare dir_publisher
		self.movement_publisher = self.create_publisher(Int32MultiArray, 'movement_coord', 10)
		#self.timer = self.create_timer(0.5, self.publish_command)


	

	def _dir_callback(self, msg):
		global x1
		global r
		global got_dir
		dir_data = msg.data
		x1 = dir_data[0]
		r = dir_data[1]
		x1 = float(x1)
		r = float(r)
		got_dir = 1
		self.pub_coord()

	def _lidar_callback(self,msg):
		global lidar_data
		global got_lidar
		lidar_data = msg.ranges
		default_range = 20
		for i in range(len(lidar_data)):
			if math.isnan(lidar_data[i]):
				lidar_data[i] = default_range
		got_lidar = 1
		self.pub_coord()

	def pub_coord(self):
		global x1
		global r
		global got_dir
		global got_lidar
		if got_dir == 1 and got_lidar == 1:
			#logic
			# Set direction. Positive is angled right, negative is angled left
			direction = 1
			if x1 > 328/2:
				direction = -1

			# Perpendicular distance in pixels to center and outer edges of object
			x1_bar = 328/2 - x1 # units of pixel values. Describes perpendicular distance from x_axis of robot
			x1_bar = x1_bar*direction

			x2_bar = x1_bar - r # result is perp distance from center axis to edge of object. In pixels
			x3_bar = x1_bar + r # result is perp distance from center axis to other edge of object. In pixels

			# angular offset from x axis to center and outer edges of object
			theta1 = x1_bar * (62.2/2) / (328/2) # result is angular displacement from x_axis of robot in degrees
			theta2 = x2_bar * (62.2/2) / (328/2) # result is angular displacement from x_axis of robot in degrees
			theta3 = x3_bar * (62.2/2) / (328/2) # result is angular displacement from x_axis of robot in degrees

			# this part will access lidar range data for specific angles
			angular_resolution = 1.62 # degrees
			angle_index1 = int(theta1 / angular_resolution) # index that refers to angle of center of object
			angle_index2 = int(theta2 / angular_resolution) # index that refers to angle of edge of object
			angle_index3 = int(theta3 / angular_resolution) # index that refers to angle of other edge of object

			# Check all distances in that range and pick the closest (smallest one)
			length = abs(angle_index3)-abs(angle_index2)+1
			range_at_angle = [None] * length
			for i in range(0, length, 1):
				range_at_angle[i] = lidar_data[angle_index2+i]

			# Find desired distance and angle to output
			x_d = min(range_at_angle)
			index = range_at_angle.index(x_d)
			ang_err = direction*(angle_index2+index) * angular_resolution # angular error in degrees
			
			# publish direction
			msg = Int32MultiArray()
			msg.data = [int(x_d),int(ang_err)]
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
