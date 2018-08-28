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



#parameters
a1 = 0.01
a2 = 0.01 
a3 = 0.01
a4 = 0.01
a5 = 0.01
a6 = 0.01
DT = 0.03

Q = numpy.array([
				[0.1,	0,		0],
				[0,		0.2, 	0],
				])

#initial guess
theta = 0.5
mu = numpy.array([
				[0],
				[0],
				[0]
				])

P = numpy.array([
				[0.4,	0,		0],
				[0,		0.3, 	0],
				[0, 	0, 		0.3]
				])


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
v = 0
w = 0


m = [
			[1.03, -2.52],
			[1.04, 1.47],
			[4.5, 0.952],
			[4.99, -2.89],
			[8.57, -2.79],
			[7.5, 1.07]
			
			]

#create a publisher
#pub0 = rospy.Publisher('publish_odom_pose',pose_msg, queue_size =10)
pub = rospy.Publisher('publish_ekf_localization_known_correspondences',pose_msg, queue_size =10)

def odomCallback(msg):
	global x_odom_rec, y_odom_rec
	global a1, a2, a3, a3, a4, a5, a6, DT, v, w
	global pub
	global mu, P

	x = msg.pose.pose.position.x
	y = msg.pose.pose.position.y
	quat = msg.pose.pose.orientation
	(roll,pitch,yaw) = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])




	x_odom_rec.append(x)
	y_odom_rec.append(y)




def odomFiteredCallback(msg):
	global x_filtered, y_filtered, x_filtered_rec, y_filtered_rec, yaw_filtered
	global mu, P

	x_filtered = msg.pose.pose.position.x
	y_filtered = msg.pose.pose.position.y
	quat = msg.pose.pose.orientation
	(roll,pitch,yaw_filtered) = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])


	#linear and angular velocity from odometry
	v = msg.twist.twist.linear.x 
	w = msg.twist.twist.angular.z

	#continue with table 7.2
	theta = mu[2][0]
	vOverW = v/w
	th_w_Dt = theta + (w*DT)
	Gt = numpy.array([
					[1, 0, -vOverW*math.cos(theta) + vOverW*math.cos(th_w_Dt)],
					[0, 1, -vOverW*math.sin(theta) + vOverW*math.sin(th_w_Dt)],
					[0, 0, 1]
					 ])

	eq1 = ((v*(math.sin(theta)-math.sin(th_w_Dt)))/ (w**2) ) + ( (v*math.cos(th_w_Dt)*DT)/w )
	eq2 = ((v*(math.cos(theta)-math.cos(th_w_Dt)))/ (w**2) ) + ( (v*math.sin(th_w_Dt)*DT)/w )
	Vt = numpy.array([
					[ (-math.sin(theta)+math.sin(th_w_Dt))/w, eq1],
					[ (math.cos(theta)-math.cos(th_w_Dt))/w, eq2],
					[0, DT]
					])
	
	Mt = numpy.array([
					[a1*(v**2)+a2*(w**2), 	0],
					[0, 					a3*(v**2) + a4*(w**2)]
					])
	
	wrapedAngle = ((w*DT) + numpy.pi) % (2*numpy.pi) - numpy.pi
	poseModel = numpy.array([
							[-vOverW*math.sin(theta) + vOverW*math.sin(th_w_Dt)],
							[vOverW*math.cos(theta) - vOverW*math.cos(th_w_Dt)],
							[wrapedAngle]
							])
	muBar = mu + poseModel

	Pm = numpy.dot( numpy.dot(Gt,P), numpy.transpose(Gt) ) + numpy.dot( numpy.dot(Vt,Mt), numpy.transpose(Vt) )

	mu = muBar
	P = Pm
	x_est_rec.append(muBar[0][0])
	y_est_rec.append(muBar[1][0])



	x_filtered_rec.append(x_filtered)
	y_filtered_rec.append(y_filtered)
	



def line_extract_estimate(msg):
	global x_est_rec, y_est_rec

	line_features = msg.line_segments
	feature_len = len(line_features)

	#since we don't have know which landmarks the robot is seeing, therefore
	#we have to calculate those correspondence, another words, we need to know
	#which landmark in the given map the robot is seeing. To do this, we 
	#will calculate the euclidean distance between all the coordiantes in the 
	#given map and each of the observation. This way, the shortest distance between
	#the obervation and the map will likely to be the landmark that the robot is 
	#seeing.

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
			bearing_wrt_x =  math.atan2(disty, distx)


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


			#continue on ch7 table 7.2 step 9






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
	plt.plot(x_est_rec,y_est_rec,'b-')
	plt.plot(x_odom_rec, y_odom_rec, 'r-')
	plt.plot(x_filtered_rec, y_filtered_rec, 'g')
	
		
	plt.ylabel("y")
	plt.xlabel("x")

	plt.savefig('/home/husky/catkin_ws/plots/ekf_localization_known_correspondences.pdf')

def main():
	#initialize node
	#rospy.init_node("publish_odom_pose", anonymous=True)
	rospy.init_node("publish_ekf_localization_known_correspondences", anonymous=True)

	#sub to odometry node (/odom)
	rospy.Subscriber("/husky_velocity_controller/odom", Odometry, odomCallback)

	rospy.Subscriber("/odometry/filtered", Odometry, odomFiteredCallback)

	#sub to line extraction
	rospy.Subscriber('line_segments', LineSegmentList, line_extract_estimate)



if __name__ == '__main__':

	
	
	print("starting ekf_localization_known_correspondences algorithm...")
	main()
	rospy.spin() 	#keep the program running until ctrl-c
	plot_data()		#plot and saves it after exiting the program 