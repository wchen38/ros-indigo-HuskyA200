#!/usr/bin/env python
import rospy
import math
from husky_hallway_pid.msg import pid_input
from geometry_msgs.msg import Twist

MAX_POS_STEERING_ANGLE = 30*math.pi/180
MAX_NEG_STEERING_ANGLE = -30*math.pi/180

kp = 14.0
ki=0
kd = 0.09
VELOCITY = 0.5
prev_error = 0
integral = 0
#steering_angle = 18

pub = rospy.Publisher('husky_velocity_controller/cmd_vel', Twist, queue_size=10)

def controlCallback(msg):
	global kp
	global kd, ki
	global prev_error, integral
	global MAX_POS_STEERING_ANGLE
	global MAX_NEG_STEERING_ANGLE

	curr_error = msg.pid_error
	integral = integral + curr_error
	if curr_error != 0.0:
		integral = integral + curr_error
		res = prev_error - curr_error
		steering_angle = (kp*curr_error + ki*integral + kd*res) * (math.pi/180)
		
	prev_error = curr_error

	# if steering_angle > MAX_POS_STEERING_ANGLE:
	# 	steering_angle = MAX_POS_STEERING_ANGLE
	# if steering_angle < MAX_NEG_STEERING_ANGLE:
	# 	steering_angle = MAX_NEG_STEERING_ANGLE

	control_msg = Twist()
	control_msg.linear.x = VELOCITY
	#if abs(steering_angle) <= 2.79253:
	control_msg.angular.z = steering_angle
	
	pub.publish(control_msg)

if __name__ == '__main__':
	
	print("Listening to error for PID...")
	kp = input("Enter Kp Value: ")
	ki = input("Enter Ki Value: ")
	kd = input("Enter Kd Value: ")
	rospy.init_node('pid_controller', anonymous=True)
	rospy.Subscriber("pid_error", pid_input, controlCallback)
	rospy.spin()