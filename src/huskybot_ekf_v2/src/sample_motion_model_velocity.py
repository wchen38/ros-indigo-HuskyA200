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




a1 = 0.01
a2 = 0.01 
a3 = 0.01
a4 = 0.01
a5 = 0.1
a6 = 0.1
DT = 0.03

#inital guess
estX = 0
estY = 0
theta = 0



x_filtered = 0
y_filtered = 0
yaw_filtered = 0
x_odom_rec = []
y_odom_rec = []
x_filtered_rec = []
y_filtered_rec = []
x_est_rec = []
y_est_rec = []
heading_rec = []

#create a publisher
#pub0 = rospy.Publisher('publish_odom_pose',pose_msg, queue_size =10)
pub = rospy.Publisher('publish_motion_model_velocity',pose_msg, queue_size =10)

def odomCallback(msg):
	global x_odom_rec, y_odom_rec, x_est_rec, y_est_rec, estX, estY
	global a1, a2, a3, a3, a4, a5, a6, DT, theta
	global pub
	
	x = msg.pose.pose.position.x
	y = msg.pose.pose.position.y
	quat = msg.pose.pose.orientation
	(roll,pitch,yaw) = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])


	#linear and angular velocity from odometry
	v = msg.twist.twist.linear.x 
	w = msg.twist.twist.angular.z

	#table 5.3 chapter 5
	vHat = v + numpy.random.normal(0, a1*(v**2) + a2*(w**2)) 
	wHat = w + numpy.random.normal(0, a3*(v**2) + a4*(w**2)) 
	lamdaHat = numpy.random.normal(0, a5*(v**2) + a6*(w**2)) 


	temp = theta + wHat*DT
	estX = estX - (vHat/wHat)*math.sin(theta) + (vHat/wHat)*math.sin(temp)
	estY = estY - (vHat/wHat)*math.cos(theta) + (vHat/wHat)*math.cos(temp)
	theta = theta + wHat*DT + lamdaHat*DT
	#wrap angle between -pi and pi
	#theta = (theta + numpy.pi) % (2*numpy.pi) - numpy.pi
	print theta 

	x_odom_rec.append(x)
	y_odom_rec.append(y)
	x_est_rec.append(estX)
	y_est_rec.append(estY)

	estPose = pose_msg(estX,estY, theta)
	pub.publish(estPose)



def odomFiteredCallback(msg):
	global x_filtered, y_filtered, x_filtered_rec, y_filtered_rec
	global yaw_filtered

	x_filtered = msg.pose.pose.position.x
	y_filtered = msg.pose.pose.position.y
	quat = msg.pose.pose.orientation
	(roll,pitch,yaw_filtered) = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])

	x_filtered_rec.append(x_filtered)
	y_filtered_rec.append(y_filtered)


def plot_data():
	# Plotting the predicted and updated state estimates as well as the uncertainty ellipse to see if 
	# filter is behaving as expected. 
	global heading_rec

	fig = plt.figure(1)
	ax = fig.gca()
	plt.axis('equal')
	ax1 = plt.gca()

	# Updated state estimate: 

	plt.ion()
	plt.show()

	# Update is plotted as blue points. 
	plt.plot(x_est_rec,y_est_rec,'b-')
	plt.plot(x_odom_rec, y_odom_rec, 'r-')
	plt.plot(x_filtered_rec, y_filtered_rec, 'g')
	
		
	plt.ylabel("y")
	plt.xlabel("x")

	plt.savefig('/home/husky/catkin_ws/plots/sample_motion_model_velocity.pdf')

def main():
	#initialize node
	#rospy.init_node("publish_odom_pose", anonymous=True)
	rospy.init_node("sample_motion_model_velocity", anonymous=True)

	#sub to odometry node (/odom)
	rospy.Subscriber("/husky_velocity_controller/odom", Odometry, odomCallback)

	rospy.Subscriber("/odometry/filtered", Odometry, odomFiteredCallback)



if __name__ == '__main__':

	
	
	print("starting sample_motion_model_velocity algorithm...")
	main()
	rospy.spin() 	#keep the program running until ctrl-c
	plot_data()		#plot and saves it after exiting the program 