#!/usr/bin/env python
import rospy
import math
import numpy
import time
import matplotlib.pyplot as plt

from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from laser_line_extraction.msg import LineSegmentList
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from huskybot_ekf.msg import *
from tf.transformations import euler_from_quaternion


#create a publisher
pub = rospy.Publisher('publish_odom_pose',pose_msg, queue_size =10)


def odomCallback(msg):
	global pub
	
	x = msg.pose.pose.position.x
	y = msg.pose.pose.position.y
	quat = msg.pose.pose.orientation
	(roll,pitch,yaw) = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])

	odom_pose = pose_msg(x,y, yaw)
	pub.publish(odom_pose)

def main():
	#initialize node
	rospy.init_node("publish_odom_pose", anonymous=True)


	#sub to odometry node (/odom)
	rospy.Subscriber("/husky_velocity_controller/odom", Odometry, odomCallback)



if __name__ == '__main__':

	
	
	print("starting publishing odom pose...")
	main()
	rospy.spin() 	#keep the program running until ctrl-c