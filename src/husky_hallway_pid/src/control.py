#!/usr/bin/env python
import rospy
from husky_hallway_pid.msg import pid_input
from geometry_msgs.msg import Twist

kp = 14.0
kd = 0.09

pub = rospy.Publisher('husky_velocity_controller/cmd_vel', Twist, queue_size=10)

def controlCallback(msg):
	global kp

	error = msg.pid_error
	steering_angle = error*kp

	control_msg = Twist()
	control_msg.linear.x = 0.5
	control_msg.angular.z = steering_angle
	pub.publish(control_msg)

if __name__ == '__main__':
	
	print("Listening to error for PID...")
	kp = input("Enter Kp Value: ")
	kd = input("Enter Kd Value: ")
	rospy.init_node('pid_controller', anonymous=True)
	rospy.Subscriber("pid_error", pid_input, controlCallback)
	rospy.spin()