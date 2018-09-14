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
from sensor_msgs.msg import Imu
from huskybot_ekf.msg import *
from tf.transformations import euler_from_quaternion

w = 0
a = 0
dt = 0.02
x_est_rec = []
y_est_rec = []
x_odom_rec = []
y_odom_rec = []
x_filtered_rec = []
y_filtered_rec = []

#parameters
alpha1 = 0.5
alpha2 = 0.4
alpha3 = 0.1
alpha4 = 0.1

#covariance Q, measurement noise (eq. 7.15)
#temp1 = [0.9**2, 0.9**2, 0.5**2, 0.01**2];
temp1 = [0.9**2, 0.9**2, 0.5**2, 0.01**2];
Qt = numpy.diag(temp1); 

#initial guess of the pose
X = numpy.array([
				[0],
				[0],
				[0],
				[0],
				[0],
				[0]
				])
Xm = X
#initial guess of the covariance of the state vector
cov = [0.01**2, 0.01**2, 0.01**2, 0.01**2, 0.01**2, 0.01**2];
P = numpy.diag(cov);
Pm = P;

#Ht is a Jacobian of the measurement model (eq. 7.14)
# map the a priori state x_{k | k-1} into the observed space which is 
#the measurement
Ht = numpy.array([
				[0, 0, 0, 1, 0, 0],
     			[0, 0, 0, 0, 1, 0],
     			[0, 0, 0, 0, 1, 0],
     			[0, 0, 0, 0, 0, 1]
     			]);

#the motion noise to be mapped into state space (eq. 7.11)
Vt = numpy.array([
				[0, 0],
      			[0, 0],
      			[1, 0],
      			[0, 1],
      			[1, 0],
      			[0, 1]
      			]);

Xrec = []; #record all the pose inorder to plot later

def odomCallback(msg):
	global pub, w, a
	global Qt, Vt, Ht, X, P, Pm, Xm
	global x_est_rec, y_est_rec, x_odom_rec, y_odom_rec


	x = msg.pose.pose.position.x
	y = msg.pose.pose.position.y
	quat = msg.pose.pose.orientation
	(roll,pitch,yaw) = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
	#linear velocity
	v = msg.twist.twist.linear.x
	odom_w = msg.twist.twist.angular.z
	
	theta = X[2] 
	#-------------------------------prediction---------------------------
    #The Jacobian from the motion model (eq. 7.8)
	Gt = numpy.array([
    				[1, 0, -v*dt*math.sin(theta),   dt*math.cos(theta), 0, 0],
          			[0, 1, dt*math.cos(theta),    	dt*math.sin(theta), 0, 0],
          			[0, 0, 1,                		0,            		dt, 0],
          			[0, 0, 0,                		1,            		0, dt],
          			[0, 0, 0,               		0,             		1, 0],
          			[0, 0, 0,                		0,             		0, 1]
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
						[0],
						[0],
						[0]
						]);

	#predict the covarence
	Pm = MatMul(MatMul(Gt,P), numpy.transpose(Gt)) + MatMul(MatMul(Vt,Mt), numpy.transpose(Vt));
	
	
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

	#print "^^^^^^^^^^^^^^^^^^^^^^^"
	#print (MatMul(Pm, numpy.transpose(Ht))).shape
	#print "---------"
	#print (numpy.linalg.inv(innovation_cov)).shape
	#measurement model
	z = numpy.array([
					[v],
					[odom_w],
					[w],
					[a]
					])
      
	#expected measurements from our prediction
	z_exp = numpy.array([
						[Xm[3]],
						[Xm[4]],
						[Xm[4]],
						[Xm[5]]
						])
	
	#innovation, difference between what we observe and what we expect
	innovation = z - z_exp;
	
	#update the pose
	X = Xm + MatMul(Kt, numpy.squeeze(innovation));
	X[3] = ( X[3] + numpy.pi) % (2 * numpy.pi ) - numpy.pi
      
	#update the covarence
	P =MatMul((numpy.identity(6) - MatMul(Kt,Ht)), Pm);

	P = Pm
	X = Xm 
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
	plt.plot(x_est_rec,y_est_rec,'b')
	plt.plot(x_odom_rec, y_odom_rec, 'r')
	plt.plot(x_filtered_rec, y_filtered_rec, 'g')
	
	
	plt.ylabel("y")
	plt.xlabel("x")

	plt.savefig('/home/husky/catkin_ws/plots/ekf_odom_imu.pdf')


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



if __name__ == '__main__':

	
	
	print("start fusing odometry and imu to estimate pose...")
	main()
	rospy.spin() 	#keep the program running until ctrl-c
	plot_data()		#plot and saves it after exiting the program 