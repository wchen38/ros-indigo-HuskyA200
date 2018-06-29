#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan

def callback(msg):
	#code for finding the distance between wall and robot
	#print len(msg.ranges)
	print 'value at 0 degrees:'
	print msg.ranges[0]

	print 'value at 90 degrees:'
	print msg.ranges[360]

	print 'value at 180 degrees:'
	print msg.ranges[719]

if __name__ == '__main__':
	print("Laser node started")
	rospy.init_node('dist_scan',anonymous = True)
	rospy.Subscriber("scan",LaserScan,callback)
	rospy.spin()
