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



sigR = 0.01
sigBearing = 0.01
OFFSET = 0.8

m = [
			[1.03, -2.52],
			[1.04, 1.47],
			[4.5, 0.952],
			[4.99, -2.89],
			[8.57, -2.79],
			[7.5, 1.07]
			
			]

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
pub1 = rospy.Publisher('publish_measurment_pose',pose_msg, queue_size =10)

def odomCallback(msg):
	global x_odom_rec, y_odom_rec
	
	x = msg.pose.pose.position.x
	y = msg.pose.pose.position.y
	quat = msg.pose.pose.orientation
	(roll,pitch,yaw) = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
	x_odom_rec.append(x)
	y_odom_rec.append(y)
	#odom_pose = pose_msg(x,y, yaw)
	#pub.publish(odom_pose)

def odomFiteredCallback(msg):
	global x_filtered, y_filtered, x_filtered_rec, y_filtered_rec
	global yaw_filtered
	x_filtered = msg.pose.pose.position.x
	y_filtered = msg.pose.pose.position.y
	quat = msg.pose.pose.orientation
	(roll,pitch,yaw_filtered) = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
	
	x_filtered_rec.append(x_filtered)
	y_filtered_rec.append(y_filtered)


def line_extract_estimate(msg):
	global x_filtered, y_filtered, sigR, sigBearing, pub1
	global x_est_rec, y_est_rec, yaw_filtered, heading_rec, OFFSET

	line_features = msg.line_segments

	feature_len = len(line_features)

	for k in range(0, feature_len-1):
		
		# find the midpoint of the line feature
		midx = (line_features[k].start[0] + line_features[k].end[0])/2
		midy = (line_features[k].start[1] + line_features[k].end[1])/2

		distx = (midx - x_filtered)
		#added the negative(-midy) because line_feature is in odom frame and
		#we need to transform it into base frame. If, I don't do this, then
		#the start, end point won't be aligned with the midpoint
		disty = (-midy - y_filtered)
		
		r = math.sqrt(distx**2 + disty**2)
		#only care about the features that are 3 meters away
		if(r < 3):
			
			#got the relative bearing of the landmarks in repect to the robot
			#bearing with respect to world frame -y axis
			bearing = (math.pi - math.atan2(distx, disty)) # + yaw_filtered

			#bearing with respect to world frame +x aixs
			#bearing_wrt_x =  math.atan2(disty, distx)
			

			#print "bearing: ", bearing, "yaw: ", yaw_filtered
			
			landmarkIndex = 0
			minDist = 999

			# for i in range(0, len(m)):
			for i in range(0, len(m)):

				# compare the euclidean distance of each map coordinates with 
				# midpoint of a line to find the smallest distance between map coordinates
				# and midpoint 
				mapx = m[i][0]
				mapy = m[i][1]

				distx1 = (midx - mapx)**2
				disty1 = (midy - mapy)**2
				euclidean_dist = math.sqrt(distx1 + disty1)

				#keep that index as j in table 6.5, since that will represent which landmark the 
				#laser detected


				if(euclidean_dist < minDist):
					minDist = euclidean_dist
					landmarkIndex = i

			lamdaHat = numpy.random.rand(1,1) * 2*math.pi
			 
			#continue with table 6.5
			rHat = r + numpy.random.normal(0, sigR)
			bearingHat = bearing + numpy.random.normal(0, sigBearing)

			poseX = m[landmarkIndex][0] + rHat*math.cos(lamdaHat)
			poseY = m[landmarkIndex][1] + rHat*math.sin(lamdaHat)
			heading = (math.pi - (2*bearingHat))/2
			
			#the angle between landmark and the world frame x-aixs
			fea_bearing = (heading - bearing)
			print "c: ", fea_bearing

			dty = OFFSET * math.sin(heading)
			dtx = OFFSET * math.cos(heading)
			x_est_rec.append(poseX - dtx)
			y_est_rec.append(poseY - dty)
			heading_rec.append(heading)

			mes_pose = pose_msg(poseX,poseY, heading)
			pub1.publish(mes_pose)



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
	

	
	for i in range(0, len(x_est_rec)-1):
		
		dtx = 0.1 * math.cos(heading_rec[i])
		dty = 0.1 * math.sin(heading_rec[i])
		x_start = x_est_rec[i]
		y_start = y_est_rec[i]
		x_end = x_start - dtx
		y_end = y_start + dty
		
		#decides which direction to plot the heading
		if( (heading_rec[i]>=0) and (heading_rec[i]<=math.pi/2) or ( (heading_rec[i]<-3*math.pi/2) and (heading_rec[i]>=-2*math.pi) )  ):
			x_end = x_start + dtx
			y_end = y_start + dty
		elif( (heading_rec[i]>math.pi/2) and (heading_rec[i]<=math.pi)  or ( (heading_rec[i]<-math.pi) and (heading_rec[i]>=-3*math.pi/2) ) ):
			x_end = x_start - dtx
			y_end = y_start + dty
		elif( (heading_rec[i]<0) and (heading_rec[i]>=-math.pi/2) or ( (heading_rec[i]>3*math.pi/2) and (heading_rec[i]<=2*math.pi) ) ):
			x_end = x_start + dtx
			y_end = y_start - dty
		elif(( (heading_rec[i]<-math.pi/2) and (heading_rec[i]>=-math.pi) ) or ( (heading_rec[i]>math.pi) and (heading_rec[i]<=3*math.pi/2) ) ):
			x_end = x_start - dtx
			y_end = y_start - dty
		else:
			print heading_rec[i]
		

		plt.plot([x_start, x_end],[y_start, y_end], 'g-')
		
	plt.ylabel("y")
	plt.xlabel("x")

	plt.savefig('/home/husky/catkin_ws/plots/sample_landmark_model.pdf')


def main():
	#initialize node
	#rospy.init_node("publish_odom_pose", anonymous=True)
	rospy.init_node("publish_measurment_pose", anonymous=True)

	#sub to odometry node (/odom)
	rospy.Subscriber("/husky_velocity_controller/odom", Odometry, odomCallback)

	rospy.Subscriber("/odometry/filtered", Odometry, odomFiteredCallback)

	#sub to line extraction
	rospy.Subscriber('line_segments', LineSegmentList, line_extract_estimate)



if __name__ == '__main__':

	
	
	print("starting sample_landmark_model_known_corespondence algorithm...")
	main()
	rospy.spin() 	#keep the program running until ctrl-c
	plot_data()		#plot and saves it after exiting the program 