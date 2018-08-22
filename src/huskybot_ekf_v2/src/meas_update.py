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

#PRIOR KNOWLEDGE OF STATE
meas_data = 0
ave_meas_dist = 0

predicted_pose = 0
predicted_cov = 0
pose_estimate = pose_msg()
Vt = 0 				#noise in motion model

#for plotting
x_updated = []
y_updated = []
x_odom = []
y_odom = []
x_filtered = []
y_filtered = []

DELTA_T = 0.02

#line extraction
line_features = LineSegmentList()
my_list = list()
time_counter = 0
curr_time_index = 0
flag = 0

m = [
			[0.91, -1.95],
			
			[4.35, -2.27],
			
			[7.52, -1.59],
			
			[10.1, 0.205],
			
			[11, 3.5],
			
			[9.36, 6.23],
			
			[6.66, 7.19],
			
			[3.38, 7.89],
			
			]
NUMBER_OF_LANDMARKS = len(m)


#create a publisher
pub = rospy.Publisher('state_estimate',pose_msg, queue_size =10)


def odom_state_prediction(msg):
	global DELTA_T
	global x_odom, y_odom
	global predicted_pose	#predicted pose (x,y, yaw)
	global predicted_cov	#predicted covariance 
	global Vt 				#noise in motion model


	#get the linear and angular velocity
	u = numpy.array([
		[msg.twist.twist.linear.x],
		[msg.twist.twist.angular.z]])

	rot = DELTA_T*u[0]
	halfRot = rot/2
	trans = u[1] * DELTA_T

	#get the x, y and yaw
	#convert yaw from quaternion using tf.transfromation euler_from_quaternion()
	x = msg.pose.pose.position.x
	y = msg.pose.pose.position.y
	quat = msg.pose.pose.orientation
	(roll,pitch,yaw) = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
	#print "yaw------------>>>>> ", yaw

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
	#Gt = numpy.array([[1,0,0],[0,1,0],[0,0,1]])
	Gt = numpy.array([[1,0,-math.sin(yaw+halfRot)],[0,1,math.cos(yaw+halfRot)],[0,0,1]])
	# Calculated the total uncertainty of the predicted state estimate
	predicted_cov = Gt*predicted_cov*numpy.transpose(Gt)+Vt

	#store all the odom x,y values for plot after exiting the program
	x_odom.append(x)
	y_odom.append(y)



def meas_update_step(msg):

	start_time = time.time()
	global curr_time_index, m, NUMBER_OF_LANDMARKS
	global 	x_updated
	global y_updated
	global pub
	global predicted_cov	#predicted covariance 
	test_count = 0
	old_time_index = 0
	if( (len(my_list) >= 1)):
		#print "curr_time_inex: ", curr_time_index
		
		curr_feature = my_list[curr_time_index]


		#print "******************"
		feature_len = len(curr_feature)

		feature1_temp = curr_feature[0].end
		feature2_temp = curr_feature[feature_len-1].end

		feature1 = [feature1_temp[0], feature1_temp[1]*-1]
		feature2 = [feature2_temp[0], feature2_temp[1]*-1]

		point_fea = numpy.array([feature1,
								feature2
								])
		print "*******************"
		#print point_fea
		for i in range(0,len(point_fea)):
			
			
			maxJ = 0
			landmarkIndex = 0
			H1 = numpy.zeros((2, 3, NUMBER_OF_LANDMARKS))
	 		expected_meas1 = numpy.zeros((2,1, NUMBER_OF_LANDMARKS))
	 		innovation_cov1 = numpy.zeros((2,2, NUMBER_OF_LANDMARKS))


			#find the slop of the line feature
			robot_x = predicted_pose.x
			robot_y = predicted_pose.y
			feature_start = curr_feature[i].start
			feature_end = curr_feature[i].end

			p1 = [feature_start[0], feature_start[1]*-1]
			p2 = [feature_end[0], feature_end[1]*-1]
			
			print "p1: ", p1
			print "p2: ", p2

			#slope = (y2 - y1)/(x2 - x1)
			feature_slope = (p2[1] - p1[0]) / (p2[0] - p1[0])
			#print "feature_slope = ", feature_slope
			# slope intercept form y = mx+b => b= y - mx
			feature_intercept = p1[1] - feature_slope*p1[0]
			#print "feature_intercept = ", feature_intercept
			#slope of the perpendicular line, negative reciprocal of the original line
			perp_line_slope = -1/feature_slope
			#print "perp_line_slope = ", perp_line_slope

			perp_line_intercept = 0 - perp_line_slope*0

			#set the two line equation equal to each other and find x which is where 
			#both line cross
			point_feature_x = (perp_line_intercept - feature_intercept) / (feature_slope - perp_line_slope)
			point_feature_y = (perp_line_slope*point_feature_x) + perp_line_intercept
			
			xx1 = point_feature_x - robot_x
			yy1 = point_feature_y - robot_y

			feature_range1 = math.sqrt(math.pow(xx1,2)  + math.pow(yy1,2))
			feature_bearing1 = math.atan2(yy1,xx1)
			
			#print "range1: ", feature_range1
			#print "bearing1: ", feature_bearing1
			#----------------------------------new code for end point feather range and bearing --------------
			xx = point_fea[i][0]-robot_x
			yy = point_fea[i][1]-robot_y
			feature_range = math.sqrt(math.pow(xx,2)  + math.pow(yy,2))
			feature_bearing = math.atan2(yy,xx)
			#print "range: ", feature_range
			#print "bearing: ",feature_bearing
			

	 		line_radius = curr_feature[i].radius
			line_angle = curr_feature[i].angle
			print "radius: ", line_radius
			print "angle: ", line_angle
			obs_meas = numpy.array([[line_radius],
									[line_angle]
									])
			#print "******************"
			for k in range(0, len(m)):

				
				
				mkx = m[k][0]
				mky = m[k][1]
				posex = predicted_pose.x
				posey = predicted_pose.y
				poseth = predicted_pose.theta

				distx = math.pow((mkx - posex), 2);
				disty = math.pow((mky - posey), 2);
				
				q = distx + disty
				angle = math.atan2(mky-posey, mkx-posex) - poseth

				#print "--------------------"
				#print "q: ", q
				#print "angle: ", angle

				expected_meas1[:,:,k] = numpy.array([[math.sqrt(q)],
											[angle]
											])

				H1[:,:,k] = numpy.array([[-(mkx-posex)/math.sqrt(q), -(mky-posey)/math.sqrt(q) , 0],
								[(mky-posey)/q, -(mkx-posex)/q, -1]
								])
				Qt = 0.5


				temp8 = numpy.dot(numpy.squeeze(H1[:,:,k]), predicted_cov)
				
				innovation_cov1[:,:,k] = numpy.dot(temp8, numpy.transpose(numpy.squeeze(H1[:,:,k]))) + Qt

				innovation1 = obs_meas - expected_meas1[:,:,k]
				
				#print obs_meas
				#print expected_meas1[:,:,k]
				#print "-------------------"
				#print innovation1
				
				temp5 = numpy.dot(-1/2*numpy.transpose(innovation1), numpy.linalg.inv(numpy.squeeze(innovation_cov1[:,:,k])))
				temp6 = numpy.dot(temp5, innovation1)
				#print "innovation cov1: "
				#print innovation_cov1[:,:,k]
				temp7 = math.sqrt(numpy.linalg.det(2*math.pi*numpy.squeeze(innovation_cov1[:,:,k])))
				thisJ =temp7 * math.exp(temp6[0][0])

				#pick the index that has most likelyhood to be the landmark 
				if(thisJ > maxJ):
					maxJ = thisJ
					landmarkIndex = k

			temp11 = numpy.dot(predicted_cov, numpy.transpose(numpy.squeeze(H1[:,:,k])))
			kalman_gain = numpy.dot(temp11,numpy.linalg.inv(innovation_cov1[:,:,k]))
			updated_pose =  numpy.array([predicted_pose.x, predicted_pose.y, predicted_pose.theta]) + numpy.dot(kalman_gain, innovation1)
			#print "shape: " 
			#print 
			#predicted_cov= (numpy.identity(3) - numpy.dot(kalman_gain,H1[:,:,k]))*predicted_cov
			#print "prediected_cov"
			#print predicted_cov
		#print "******************"
		curr_time_index = time_counter - 1
		
		pose_estimate = pose_msg(updated_pose[0], updated_pose[1], updated_pose[2])

		#publish the estiamted pose
		pub.publish(pose_estimate)


		#store all the estimated x,y values for plot after exiting the program
		x_updated.append(pose_estimate.x)
		y_updated.append(pose_estimate.y)
		elapsed_time = time.time() - start_time
		#print "elapsed time: ", elapsed_time
		


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

def line_extract_estimate(msg):
	global line_features, my_list, time_counter, flag
	

	#add the features to a list and keep track of the time
	#so the current features can be in sync with the python code
	#print "***************"

	line_features = msg.line_segments

	my_list.append(line_features)
	time_counter += 1
	temp1 = line_features[0].radius
	temp2 = line_features[0].angle
	currF = my_list[0]


	if False:
		print "radius: ", (currF[0]).start
		print "angle: " ,currF[0].end
		print "x: ", (currF[0]).radius * math.sin(-1*currF[0].angle)
		print "y: ", (currF[0]).radius * math.cos(currF[0].angle)
		
		print "-----------------"

		print "radius: ", currF[1].start
		print "angle: " ,currF[1].end
		print "x: ", currF[1].radius * math.sin(-1*currF[0].angle)
		print "y: ", currF[1].radius * math.cos(currF[0].angle)

		print "-----------------"

		print "radius: ", currF[2].start
		print "angle: " ,currF[2].end
		print "x: ", currF[2].radius * math.sin(-1*currF[2].angle)
		print "y: ", currF[2].radius * math.cos(currF[2].angle)

		print "-----------------"

		print "radius: ", currF[3].start
		print "angle: " ,currF[3].end
		print "x: ", currF[3].radius * math.sin(-1*currF[3].angle)
		print "y: ", currF[3].radius * math.cos(currF[3].angle)
		
		print "-----------------"

		print "radius: ", currF[4].start
		print "angle: " ,currF[4].end
		print "x: ", currF[4].radius * math.sin(-1*currF[4].angle)
		print "y: ", currF[4].radius * math.cos(currF[4].angle)

		print "-----------------"

		print "radius: ", currF[5].start
		print "angle: " ,currF[5].end
		print "x: ", currF[5].radius * math.sin(-1*currF[5].angle)
		print "y: ", currF[5].radius * math.cos(currF[5].angle)

		print "-----------------"
		print "radius: ", currF[6].start
		print "angle: " ,currF[6].end
		print "x: ", currF[6].radius * math.sin(-1*currF[6].angle)
		print "y: ", currF[6].radius * math.cos(currF[6].angle)

		print "len of obs: ", len(currF)
	#print "***************"

def get_filtered_pose(msg):
	global x_filtered, y_filtered
	
	x_filtered.append(msg.pose.pose.position.x)
	y_filtered.append(msg.pose.pose.position.y)
	quat = msg.pose.pose.orientation
	(roll,pitch,yaw) = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
	#print "filtered yaw ----> ", yaw



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
	plt.plot(x_odom, y_odom, 'r')
	plt.plot(x_filtered, y_filtered, 'g')
	plt.ylabel("y")
	plt.xlabel("x")

	plt.savefig('./plots/plot1.pdf')



def main():
	#initialize node
	rospy.init_node("meas_update", anonymous=True)

	#sub to the laser scan node (/scan)
	rospy.Subscriber('scan', LaserScan, laser_scan_estimate)

	#sub to odometry node (/odom)
	rospy.Subscriber("/husky_velocity_controller/odom", Odometry, odom_state_prediction)

	#sub to line extraction
	rospy.Subscriber('line_segments', LineSegmentList, line_extract_estimate)

	#Did 'rostopic hz /scan' to find the publishing frequency, which is about 40hz
	#this rate will determine how fast the filter can be run.

	#create a timer to call the measurement update function.
	#this is being called at 40hz or 0.025 ms,  30hz or 33.3ms = 0.033
	rospy.Timer(rospy.Duration(0.033), meas_update_step, oneshot=False)

	rospy.Subscriber('/odometry/filtered', Odometry, get_filtered_pose)





if __name__ == '__main__':

	
	
	print("starting measurement update...")
	main()
	rospy.spin() 	#keep the program running until ctrl-c
	plot_data()		#plot and saves it after exiting the program 

	