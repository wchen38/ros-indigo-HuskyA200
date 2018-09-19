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
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import Imu
from huskybot_ekf.msg import *
from tf.transformations import euler_from_quaternion

w = 0
a = 0
dt = 0.020
x_est_rec = []
y_est_rec = []
x_odom_rec = []
y_odom_rec = []
x_filtered_rec = []
y_filtered_rec = []
gps_x = 0
gps_y = 0
gps_x_rec = []
gps_y_rec = []
orign_x = 0
orign_y = 0 
gps_flag = 0
true_x = 0
true_y = 0
true_x_rec = []
true_y_rec = []
gps_status = -1
GPS_HAS_FIX = 0

#parameters
alpha1 = 5
alpha2 = 5
alpha3 = 5
alpha4 = 5




#initial guess of the pose
X = numpy.array([
				[0],
				[0],
				[0],
				[0]
				])
Xm = X
#initial guess of the covariance of the state vector
#cov = [0.01**2, 0.01**2, 0.01**2, 0.01**2];
cov = [5**2, 5**2, 0.01**2, 0.01**2];
P = numpy.diag(cov);
Pm = P;

Qtmp = [0.1**2, 0.1**2, math.radians(1.0)**2, 1.0**2];
QQ = numpy.diag(Qtmp);



#the motion noise to be mapped into state space (eq. 7.11)
Vt = numpy.array([
				[1, 0],
      			[1, 0],
      			[0, 1],
      			[0, 1]
      			]);

Xrec = []; #record all the pose inorder to plot later

def odomCallback(msg):
	global pub, w, a
	global Qt, Vt, Ht, X, P, Pm, Xm
	global x_est_rec, y_est_rec, x_odom_rec, y_odom_rec
	global gps_x, gps_y, true_x, true_y, GPS_HAS_FIX

	prev_t = msg.header.stamp.secs + (msg.header.stamp.nsecs/1000000000.0)
	#print prev_t

	x = msg.pose.pose.position.x
	y = msg.pose.pose.position.y
	quat = msg.pose.pose.orientation
	(roll,pitch,yaw) = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
	#linear velocity
	v = msg.twist.twist.linear.x
	odom_w = msg.twist.twist.angular.z
	
	#print gps_x, gps_y
	gps_x_rec.append(gps_x)
	gps_y_rec.append(gps_y)

	true_x_rec.append(true_x)
	true_y_rec.append(true_y)

	theta = X[2] 


	#-------------------------------prediction---------------------------
    #The Jacobian from the motion model (eq. 7.8)
	Gt = numpy.array([
    				[1, 0, -v*dt*math.sin(theta),   0],
          			[0, 1, dt*math.cos(theta),      0],
          			[0, 0, 1,                 		dt],
          			[0, 0, 0,                		1]
          			]);

    #To derive the covariance of the additional motion noise nor(0, R)
    #we first determine the covariance matrix of the noise in control
    #space
	Mt = numpy.array([
					[alpha1*(v**2) + alpha2*(a**2),   0],
            		[0,                               alpha3*(v**2) + alpha4*(a**2)]
            		]);
     
	#motion model, unicycle model, to predict the pose
	Xm = X + numpy.array([
						[v*math.cos(theta)*dt],
						[v*math.sin(theta)*dt],
						[w*dt],
						[0]
						]);

	#predict the covarence
	#Pm = MatMul(MatMul(Gt,P), numpy.transpose(Gt)) + MatMul(MatMul(Vt,Mt), numpy.transpose(Vt));
	Pm = MatMul(MatMul(Gt,P), numpy.transpose(Gt)) + QQ
	
	
	
	if(gps_status == GPS_HAS_FIX):
		#print "odom + gypro + gps ekf"
		#covariance Q, measurement noise (eq. 7.15)
		#temp1 = [0.9**2, 0.9**2, 0.5**2, 0.01**2];
		temp1 = [1.0, math.radians(40)**2, 1, 1];
		Qt = Qt = numpy.diag(temp1);
		#Qt = 1**2

		#Ht is a Jacobian of the measurement model (eq. 7.14)
		# map the a priori state x_{k | k-1} into the observed space which is 
		#the measurement
		Ht = numpy.array([
						[1, 0, 0, 0],
						[0, 1, 0, 0],
						[0, 0, 0, 1],
						[0, 0, 0, 1]
		     			]);

		
		#-------------update-------------------------------
		#innovation_cov: predict how much we should trust the measurement 
		#based on the a priori error covariance matrix P_{k | k-1} and 
		#the measurement covariance matrix R

		innovation_cov = MatMul(MatMul(Ht,Pm),numpy.transpose(Ht)) + Qt; 
		#print innovation_cov
		#tt = MatMul(Pm,Ht)
		#tt1 = numpy.reshape(tt)
		#print tt1.shape
		#The Kalman gain is used to to indicate how much we trust the innovation
		Kt = MatMul(MatMul(Pm, numpy.transpose(Ht)),numpy.linalg.inv(innovation_cov));

		#measurement model
		z = numpy.array([
						[gps_x],
						[gps_y],
						[odom_w],
						[w]
						])
		
	
		#expected measurements from our prediction

		z_exp_temp = numpy.array([
							[Xm[0]],
							[Xm[1]],
							[Xm[3]],
							[Xm[3]]
							])
		z_exp = numpy.squeeze(z_exp_temp, axis=(2,))
		#innovation, difference between what we observe and what we expect
		innovation_temp = z - z_exp;
		#print z_exp.shape
		innovation = numpy.reshape(innovation_temp, (4,1))
		#print innovation
	else:
		print "odom + gypro ekf"
		#covariance Q, measurement noise (eq. 7.15)
		#temp1 = [0.9**2, 0.9**2, 0.5**2, 0.01**2];
		temp1 = [0.9**2, 0.9**2];
		Qt = Qt = numpy.diag(temp1);
		#Qt = 1**2
		
		#Ht is a Jacobian of the measurement model (eq. 7.14)
		# map the a priori state x_{k | k-1} into the observed space which is 
		#the measurement
		Ht = numpy.array([
						[0, 0, 0, 1],
						[0, 0, 0, 1]
		     			]);
		
		#-------------update-------------------------------
		#innovation_cov: predict how much we should trust the measurement 
		#based on the a priori error covariance matrix P_{k | k-1} and 
		#the measurement covariance matrix R

		innovation_cov = MatMul(MatMul(Ht,Pm),numpy.transpose(Ht)) + Qt; 

		#tt = MatMul(Pm,Ht)
		#tt1 = numpy.reshape(tt)
		#print tt1.shape
		#The Kalman gain is used to to indicate how much we trust the innovation
		Kt = MatMul(MatMul(Pm, numpy.transpose(Ht)),numpy.linalg.inv(innovation_cov));

		#measurement model
		z = numpy.array([
						[odom_w],
						[w]
						])
	      
		#expected measurements from our prediction

		z_exp_temp = numpy.array([
							[Xm[3]],
							[Xm[3]]
							])
		z_exp = numpy.squeeze(z_exp_temp, axis=(2,))
		#innovation, difference between what we observe and what we expect
		innovation_temp = z - z_exp;
		#print z_exp.shape
		innovation = numpy.reshape(innovation_temp, (2,1))

	#print Kt.shape, innovation.shape
	#update the pose
	X = Xm + MatMul(Kt, innovation);
	X[3] = ( X[3] + numpy.pi) % (2 * numpy.pi ) - numpy.pi
      
	#update the covarence
	P =MatMul((numpy.identity(4) - MatMul(Kt,Ht)), Pm);

	Pm = P
	Xm = X 
	x_est_rec.append(X[0])
	y_est_rec.append(X[1])
	x_odom_rec.append(x)
	y_odom_rec.append(y)



def imuCallback(msg):
	global w, a 
	w = msg.angular_velocity.z
	a = msg.linear_acceleration.x

def odomFilteredCallback(msg):
	global x_filtered_rec, y_filtered_rec
	
	x = msg.pose.pose.position.x
	y = msg.pose.pose.position.y
	quat = msg.pose.pose.orientation
	(roll,pitch,yaw) = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
	x_filtered_rec.append(x)
	y_filtered_rec.append(y)

def gpsOdomCallback(msg):
	global gps_x, gps_y, gps_flag, orign_x, orign_y
	if (gps_flag == 0):
		orign_x = msg.pose.pose.position.x
		orign_y = msg.pose.pose.position.y
		gps_flag = 1

	gps_y = -(msg.pose.pose.position.x - orign_x)
	gps_x = msg.pose.pose.position.y - orign_y

def groundTruthCallback(msg):
	global true_x, true_y
	true_x = msg.pose.pose.position.x
	true_y = msg.pose.pose.position.y

def gpsCallback(msg):
	global gps_status
	gps_status = msg.status.status
	#gps_status = 1

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
	plt.plot(x_est_rec,y_est_rec,'*')
	plt.plot(x_odom_rec, y_odom_rec, 'r')
	plt.plot(x_filtered_rec, y_filtered_rec, 'r')
	plt.plot(gps_x_rec, gps_y_rec, 'g')
	#plt.plot(true_x_rec, true_y_rec, 'g')
	
	
	plt.ylabel("y")
	plt.xlabel("x")

	plt.savefig('/home/husky/catkin_ws/plots/ekf_odom_gyro_gps.pdf')


def MatMul(mat1, mat2):
	
	(mat1m, mat1n) = mat1.shape
	(mat2m, mat2n) = mat2.shape
	result = numpy.zeros((mat1m, mat2n))

	if(mat1n != mat2m):
		print "matrix dimension does not match (", mat1.shape, ")", "(",mat2.shape ,")"
		return 

	for i in range(0, mat1m): 
		total = 0
		for j in range(0, mat2n):
			total = numpy.dot(mat1[i,:], mat2[:,j])
			result[i,j] = total

	return result	

def main():
	#initialize node
	rospy.init_node("publish_odom_imu_ekf_pose", anonymous=True)

	#sub to raw odometry node (/odom)
	rospy.Subscriber("/husky_velocity_controller/odom", Odometry, odomCallback)

	#sub to simulated imu data
	rospy.Subscriber("/imu/data", Imu, imuCallback)

	#sub to filtered odometry node (/odom)
	rospy.Subscriber("/odometry/filtered", Odometry, odomFilteredCallback)

	#sub to positoin from gps_common package (/odom)
	rospy.Subscriber("/odom", Odometry, gpsOdomCallback)

	#sub to ground truth of the robot (/ground_truth/state)
	rospy.Subscriber("/ground_truth/state", Odometry, groundTruthCallback)

	#sub to gps (/ground_truth/state)
	rospy.Subscriber("/navsat/fix", NavSatFix, gpsCallback)



if __name__ == '__main__':

	
	
	print("start fusing odometry and imu to estimate pose...")
	main()
	rospy.spin() 	#keep the program running until ctrl-c
	plot_data()		#plot and saves it after exiting the program 