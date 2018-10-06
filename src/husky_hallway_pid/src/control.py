#!/usr/bin/env python
import rospy
import math
from husky_hallway_pid.msg import pid_input
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt
import csv
import numpy

MAX_POS_STEERING_ANGLE = 90*math.pi/180
MAX_NEG_STEERING_ANGLE = -90*math.pi/180

kp = 0  #80
ki = 0	#2
kd = 0	#0.9
VELOCITY = 4
prev_error = 0
integral = 0
DT = 0.02
#steering_angle = 18

angle_rec = []
counter = 0

true_x_rec = []
true_y_rec = []

pub = rospy.Publisher('husky_velocity_controller/cmd_vel', Twist, queue_size=10)

def controlCallback(msg):
	global kp, ki, kd
	global prev_error, integral, DT
	global MAX_POS_STEERING_ANGLE
	global MAX_NEG_STEERING_ANGLE
	global angle_rec, counter

	curr_error = msg.pid_error
	
	if curr_error != 0.0:
		integral = integral + curr_error 
		#res = (prev_error - curr_error)
		res = (prev_error - curr_error)
		steering_angle = (kp*curr_error + ki*integral + kd*res) * (math.pi/180)

		#steering_angle = (kp*curr_error) * (math.pi/180)
		
		angle_rec.append(steering_angle) #record the angle for plotting
			
		
	prev_error = curr_error

	# if steering_angle > MAX_POS_STEERING_ANGLE:
	# 	steering_angle = MAX_POS_STEERING_ANGLE
	# 	print "reach here max angle"
	# if steering_angle < MAX_NEG_STEERING_ANGLE:
	# 	steering_angle = MAX_NEG_STEERING_ANGLE
	# 	print "reach here minmimum angle"

	control_msg = Twist()
	control_msg.linear.x = VELOCITY
	#if abs(steering_angle) <= 2.79253:
	control_msg.angular.z = steering_angle
	
	pub.publish(control_msg)

def robotTruthCallBack(msg):
	global true_x_rec, true_y_rec
	true_x = msg.pose.pose.position.x
	true_y = msg.pose.pose.position.y
	#print true_x, true_y
	true_x_rec.append(true_x)
	true_y_rec.append(true_y)

def plot_data():


	#fig = plt.figure(1)
	#ax = fig.gca()
	#plt.axis('equal')
	#ax1 = plt.gca()

	#with open('/home/husky/catkin_ws/plots/pcontrl.csv', mode='w') as csv_file:
		
	#	filename = ['1', '2', '3']
		#print filename
	#	writer = csv.DictWriter(csv_file, fieldnames=filename)
	#numpy.savetxt("/home/husky/catkin_ws/plots/pcontrl.txt", angle_rec, delimiter=",")

	#plt.ion()
	#plt.show()

	plt.plot(true_x_rec, true_y_rec, '*', label="PV")
	plt.plot([10,0], [-0.15,-0.15], label="SP")
	plt.plot(0, 0)

	plt.ylabel("y")
	plt.xlabel("x")
	plt.legend(loc=4)
	
	#plt.ylim(ymin=-1)
	#ax1.set_ylim([3,1])
	plt.savefig('/home/husky/catkin_ws/plots/pcontroler.pdf')



if __name__ == '__main__':
	
	print("Listening to error for PID...")
	kp = input("Enter Kp Value: ")
	ki = input("Enter Ki Value: ")
	kd = input("Enter Kd Value: ")
	rospy.init_node('pid_controller', anonymous=True)
	rospy.Subscriber("pid_error", pid_input, controlCallback)
	rospy.Subscriber("/ground_truth/state", Odometry, robotTruthCallBack)
	rospy.spin()
	plot_data()		#plot and saves it after exiting the program
