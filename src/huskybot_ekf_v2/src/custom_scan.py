#!/usr/bin/env python
import rospy
import math
import numpy
import matplotlib.pyplot as plt

from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from laser_line_extraction.msg import LineSegmentList
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from huskybot_ekf.msg import *
from tf.transformations import euler_from_quaternion

meas_data = 0
ave_meas_dist = 0

#create a publisher
pub = rospy.Publisher('state_estimate',LaserScan, queue_size =10)

def custom_laser_scan(msg):
	global meas_data
	global ave_meas_dist

	#get the array of range value from the /scan topic
	meas_data = msg.ranges

	
	sum_dist = 0
	length = 0

	#take the average of the data, check to make sure all values are valid
	#laser is scanning the right hand side in robot frame
	for i in range (699, 719):
		if str(meas_data[i] != 'nan'):
			sum_dist += meas_data[i]
			length += 1

	if length != 0:
		ave_meas_dist = sum_dist/length
		pub.publish(ave_meas_dist)
		

def main():
	#initialize node
	rospy.init_node("custom_scan", anonymous=True)

	#sub to the laser scan node (/scan)
	rospy.Subscriber('scan', LaserScan, custom_laser_scan)


if __name__ == '__main__':

	print("starting custom laser scan update...")
	try: main()
	except rospy.ROSInterruptException: pass
	
	rospy.spin() 	#keep the program running until ctrl-c
	