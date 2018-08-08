#!/usr/bin/env python
import rospy
import math
import numpy
import matplotlib.pyplot as plt

from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from huskybot_ekf.msg import *
from tf.transformations import euler_from_quaternion

#PRIOR KNOWLEDGE OF STATE
meas_data = 0
ave_meas_dist = 0

predicted_pose = 0
predicted_cov = 0
pose_estimate = pose_msg()
Vt = 0 				#noise in motion model

x_updated = []
y_updated = []

x_odom = []
y_odom = []


#create a publisher
pub = rospy.Publisher('state_estimate',pose_msg, queue_size =10)


def odom_state_prediction(msg):
	global x_odom, y_odom
	global predicted_pose	#predicted pose (x,y, yaw)
	global predicted_cov	#predicted covariance 
	global Vt 				#noise in motion model

	#get the x, y and yaw
	#convert yaw from quaternion using tf.transfromation euler_from_quaternion()
	x = msg.pose.pose.position.x
	y = msg.pose.pose.position.y
	quat = msg.pose.pose.orientation
	(roll,pitch,yaw) = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])

	#save the predicted pose into a custom message of called pose_msg
	predicted_pose = pose_msg(x,y, yaw)

	#extract the covariance (6x6) from the odometry data(x,y,z,pitch,row,yaw) and 
	#use it as our predicted covariance
	#since we only need x, y, yaw, therefore, we extract the following indexes
	odomCov = msg.pose.covariance
	Vt = numpy.array([[odomCov[0],odomCov[1],odomCov[5]], 
					[odomCov[6],odomCov[7],odomCov[11]],  
					[odomCov[30],odomCov[31],odomCov[35]]])
	

	#state transition jocobian 
	Gt = numpy.array([[1,0,0],[0,1,0],[0,0,1]])
	# Calculated the total uncertainty of the predicted state estimate
	predicted_cov = Gt*predicted_cov*numpy.transpose(Gt)+Vt

	#store all the odom x,y values for plot after exiting the program
	x_odom.append(x)
	y_odom.append(y)



def meas_update_step(msg):
	global 	x_updated
	global y_updated
	global pub
	global predicted_cov	#predicted covariance 
	expected_meas = numpy.cross(numpy.array([0, 1, 0]), numpy.array([predicted_pose.x, predicted_pose.y, predicted_pose.theta]))
	#print "-------------------------------------"
	#print ave_meas_dist
	#print expected_meas
	#measurement residual: The difference between our actual measurement and the measurement we expected to get.
	innovation = ave_meas_dist - expected_meas
	
	#measurement noise 
	Qt = 0.005

	#measurement jacobian
	H = numpy.array([[9999, 0 , 0],[0, 1, 0],[0 , 0, 9999]])

	# Innovation (or residual) covariance
	innovation_cov = H*predicted_cov*numpy.transpose(H)+Qt

	#kalman gain
	kalman_gain = predicted_cov*numpy.transpose(H)*numpy.linalg.inv(innovation_cov)

	# Updated state estimate
	updated_pose =  numpy.array([predicted_pose.x, predicted_pose.y, predicted_pose.theta]) + numpy.dot(kalman_gain, innovation)

	# Updated covariance estimate
	predicted_cov= (numpy.identity(3) - numpy.cross(kalman_gain,H))*predicted_cov

	# Package the state estimate in custom message of type 'Config'
	pose_estimate = pose_msg(updated_pose[0], updated_pose[1], updated_pose[2])

	#publish the estiamted pose
	pub.publish(pose_estimate)


	#store all the estimated x,y values for plot after exiting the program
	x_updated.append(pose_estimate.x)
	y_updated.append(pose_estimate.y)



#a simple measurement model
#get the range from the scan topic 
#throwing most data away for simplicity
def laser_scan_estimate(msg):
	global meas_data
	global ave_meas_dist

	#get the array of range value from the /scan topic
	meas_data = msg.ranges

	
	sum_dist = 0
	length = 0

	#take the average of the data, check to make sure all values are valid
	for i in range (699, 719):
		if str(meas_data[i] != 'nan'):
			sum_dist += meas_data[i]
			length += 1

	if length != 0:
		ave_meas_dist = sum_dist/length
		#print ave_meas_dist






def main():
	#initialize node
	rospy.init_node("meas_update", anonymous=True)

	#sub to the laser scan node (/scan)
	rospy.Subscriber('scan', LaserScan, laser_scan_estimate)

	#sub to odometry node (/odom)
	rospy.Subscriber("/husky_velocity_controller/odom", Odometry, odom_state_prediction)

	#Did 'rostopic hz /scan' to find the publishing frequency, which is about 40hz
	#this rate will determine how fast the filter can be run.

	#create a timer to call the measurement update function.
	#this is being called at 40hz or 0.025 ms
	rospy.Timer(rospy.Duration(0.025), meas_update_step, oneshot=False)


def plot_data():
	# Plotting the predicted and updated state estimates as well as the uncertainty ellipse to see if 
	# filter is behaving as expected. 

	fig = plt.figure(1)
	ax = fig.gca()
	plt.axis('equal')
	ax1 = plt.gca()

	# Updated state estimate: 

	plt.ion()
	plt.show()

	# Update is plotted as blue points. 
	plt.plot(x_updated,y_updated,'b*')
	plt.plot(x_odom, y_odom, 'r*')
	plt.ylabel("y")
	plt.xlabel("x")

	plt.savefig('./plots/plot1.pdf')


if __name__ == '__main__':

	
	
	print("starting measurement update...")
	main()
	rospy.spin() 	#keep the program running until ctrl-c
	plot_data()		#plot and saves it after exiting the program 

	