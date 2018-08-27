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




a1 = 0.2
a2 = 0.03 
a3 = 0.09
a4 = 0.08
a5 = 0.00
a6 = 0.00
DT = 0.03



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

xRec = []
yRec = []
thetaRec = []
timeIndex = 0

#create a publisher
#pub0 = rospy.Publisher('publish_odom_pose',pose_msg, queue_size =10)
pub = rospy.Publisher('publish_motion_model_odometry',pose_msg, queue_size =10)

def odomCallback(msg):
	global x_odom_rec, y_odom_rec, x_est_rec, y_est_rec
	global a1, a2, a3, a3, a4, a5, a6, DT
	global pub, xRec, yRec, thetaRec
	global timeIndex
	
	x = msg.pose.pose.position.x
	y = msg.pose.pose.position.y
	quat = msg.pose.pose.orientation
	(roll,pitch,yaw) = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])


	#linear and angular velocity from odometry
	v = msg.twist.twist.linear.x 
	w = msg.twist.twist.angular.z

	xRec.append(x)
	yRec.append(y)
	thetaRec.append(yaw)
	

	#table 5.3 chapter 5
	if(len(xRec) > 3):
		xCurr = xRec[timeIndex]
		xPrev = xRec[timeIndex-1]
		yCurr = yRec[timeIndex]
		yPrev = yRec[timeIndex-1]
		thetaCurr = thetaRec[timeIndex]
		thetaPrev = thetaRec[timeIndex-1]

		rot1 = math.atan2(yCurr-yPrev, xCurr-xPrev) - thetaPrev
		distx = xPrev - xCurr
		disty = yPrev - yCurr
		trans = math.sqrt(distx**2 + disty**2)
		rot2 = thetaCurr - thetaPrev - rot1

		rot1Hat = rot1 - numpy.random.normal(a1*(rot1**2) + a2*(trans**2))
		transHat = trans - numpy.random.normal(a3*(trans**2) + a4*(rot1**2) + a4*(rot2**2))
		rot2Hat = rot2 - numpy.random.normal(a1*(rot2**2) + a2*(trans**2))

		estX = x + transHat*math.cos(yaw+rot1Hat)
		estY = y + transHat*math.sin(yaw+rot1Hat)
		estHeading = yaw + rot1Hat + rot2Hat

		x_est_rec.append(estX)
		y_est_rec.append(estY)

		x_odom_rec.append(x)
		y_odom_rec.append(y)
		estPose = pose_msg(estX,estY, estHeading)
		pub.publish(estPose)

	timeIndex = timeIndex+1


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
	plt.plot(x_est_rec,y_est_rec,'b*')
	plt.plot(x_odom_rec, y_odom_rec, 'r*')
	plt.plot(x_filtered_rec, y_filtered_rec, 'g*')
	
		
	plt.ylabel("y")
	plt.xlabel("x")

	plt.savefig('/home/husky/catkin_ws/plots/sample_motion_model_odometry.pdf')


def main():
	#initialize node
	#rospy.init_node("publish_odom_pose", anonymous=True)
	rospy.init_node("publish_motion_model_odometry", anonymous=True)

	#sub to odometry node (/odom)
	rospy.Subscriber("/husky_velocity_controller/odom", Odometry, odomCallback)

	rospy.Subscriber("/odometry/filtered", Odometry, odomFiteredCallback)



if __name__ == '__main__':

	
	
	print("starting publish_motion_model_odometry algorithm...")
	main()
	rospy.spin() 	#keep the program running until ctrl-c
	plot_data()		#plot and saves it after exiting the program 