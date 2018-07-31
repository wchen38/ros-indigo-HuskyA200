#!/usr/bin/env python
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import rospy
import numpy as np 
import math

#pub = rospy.Publisher('husky_ekf_pose', pid_input, queue_size=10)
 

#def ekf_localization(hMu, hCov, u, z, m):


def odomCallback(msg):
	x = msg.pose.pose.position.x
	y = msg.pose.pose.position.y
	theta = msg.pose.pose.orientation.z

	#ekf_localization(hMu, hCov, u, z, m);

def main():
	print("Husky_ekf_localization start!!!")



	rospy.init_node('Husky_ekf_localization',anonymous = True)
	rospy.Subscriber("/husky_velocity_controller/odom",Odometry,odomCallback)
	rospy.spin()


if __name__ == '__main__':
	main()
