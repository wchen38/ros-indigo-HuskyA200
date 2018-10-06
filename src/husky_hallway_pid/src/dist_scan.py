#!/usr/bin/env python
import rospy
import math
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from husky_hallway_pid.msg import pid_input

pub = rospy.Publisher('pid_error', pid_input, queue_size=10)
dist = 0
AC = 1.5
DESIRE_DIST = 1#abs(range_180_deg - b)	# center of the hallway
def odomCallback(msg):
	global dist
	dist = dist + msg.linear.x;
	

def callback(msg):
	global AC, DESIRE_DIST
	#code for finding the distance between wall and robot 
	#at zero degrees and 180 degrees
	theta = math.pi/4						#45 degrees in radians 
	b = msg.ranges[0] 						# zero degrees
	
	a = msg.ranges[179]						# 45 degrees
	range_180_deg = msg.ranges[719] 		# 180 degrees
	#print a, b
	
	#calculating the error after driving for about 1 meter
	alpha = math.atan((a*math.cos(theta)-b)/(a*math.sin(theta)))
	AB = b*math.cos(alpha)
	CD = AB + AC*math.sin(alpha)
	error = CD - DESIRE_DIST  
	pid_msg = pid_input()
	pid_msg.pid_error = error 
	pub.publish(pid_msg)		
	dist = 0							#reset travel distance


if __name__ == '__main__':
	
	print("Laser node started to scan for distance...")
	rospy.init_node('dist_scan',anonymous = True)
	#rospy.Subscriber("husky_velocity_controller/cmd_vel",Twist,odomCallback)
	rospy.Subscriber("scan",LaserScan,callback)
	rospy.spin()
