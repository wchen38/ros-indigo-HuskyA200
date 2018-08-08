#!/usr/bin/env python
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import rospy
import numpy as np 
import math

#pub = rospy.Publisher('husky_ekf_pose', pid_input, queue_size=10)

#parameters
alp1 = 1
alp2 = 1
alp3 = 1
alp4 = 1
sigmaR = 1
sigmaT = 1
sigmaS = 1

NUMBER_OF_LANDMARKS = 6


start_angle = -1.57079637051
end_angle = 1.56643295288
dA = 0.00436332309619
inc_angle = 0;

z=0
u = np.matrix(np.zeros((3, 1)))
m = np.matrix('0.372 1.85; \
				3.95 1.95; \
				5.33 5.73; \
				2.23 -2.77; \
				7 -1.95; \
				8.36 1.08')


mu = np.matrix(np.zeros((3, 1)))
cov = np.eye(3)
dt = 0.5;
flag = 1
def ekf_localization(hMu, hCov, u, z, m):
	global dt, alp1, alp2, alp3, alp4, mu, flag, inc_angle, NUMBER_OF_LANDMARKS
 	HRec = np.zeros((2, 3, NUMBER_OF_LANDMARKS))
 	zVecRec = np.zeros((720, 2))
 	SRec = np.zeros((2,2, NUMBER_OF_LANDMARKS))
 	v = u[0]
 	w = u[1]

	#print m[0].x
	theta = hMu[2]
	#print u[1]
	G = np.matrix([
		[1, 0, (-v/w)*math.cos(theta) + (v/w)*math.cos(w*dt)],
		[0, 1, (-v/w)*math.sin(theta) + (v/w)*math.sin(w*dt)],
		[0, 0, 1]])

	V = np.matrix([
		[(-math.sin(theta) + math.sin(theta+w*dt))/w, (v*(math.sin(theta)-math.sin(theta+w*dt))/math.pow(w, 2)) + (v*math.cos(theta+w*dt)*dt)/w],
		[(math.cos(theta)-math.cos(theta+w*dt))/w, -(v*(math.cos(theta)-math.cos(theta+w*dt))/math.pow(w, 2)) + (v*math.sin(theta+w*dt)*dt)/w],
		[0, dt]])
	M = np.matrix([
		[alp1*pow(v,2) + alp2*pow(w,2), 0],
		[0,								alp3*pow(v,2) + alp4*pow(w,2)]])
	
	muBar = np.array([(-v/w)*math.sin(theta) + (v/w)*math.sin(w*dt),
					(-v/w)*math.cos(theta) + (v/w)*math.cos(w*dt),
					w*dt])
	muBar = hMu + np.reshape(muBar, (3,1)) 	

	covBar = G*hCov*G.T + np.reshape(V*M*V.T, (3,3))

	Q = np.matrix([
		[pow(sigmaR,2), 0],
		[0, pow(sigmaT, 2)]
		])
	[mHeight, mLen] = m.shape
	zLen = len(z)
	#if(flag):
	row = 0
	for i in z:
		#need to make z a 2x1 vector, add the distance and the angle here
		inc_angle = inc_angle + dA
		zVecRec[row,:] = np.array([i, inc_angle])
		zVec = zVecRec[row,:]

		maxJ = 0
		landMarkIndex = 0
		j = 0
		for k in m:
			
			mkx = k.item(0)
			mky = k.item(1)
			muBarx = muBar[0]
			muBary = muBar[1]

			q =pow(mkx-muBarx,2) + pow(mky-muBary, 2)
			zHat = np.array([math.sqrt(q),
							math.atan2(mky-muBary, mkx-muBarx)])
			
			HRec[:,:,j] = np.matrix([
						[-(mkx-muBarx)/math.sqrt(q), -(mky-muBary)/math.sqrt(q), 0],
						[(mky-muBary)/q, -(mkx-muBarx)/q, -1]
						])
			H = HRec[:,:,j]
			SRec[:,:,j] = H*covBar*H.T + Q
			S = np.matrix([
						[SRec[0,0,j], SRec[0,1,j]],
						[SRec[1,0,j], SRec[1,1,j]]
						])
		
			res = zVec - zHat
			resT = np.transpose(zVec - zHat)
			resT = np.reshape(resT, (2,1))
			
			temp = (-1/2) *res * np.linalg.inv(S) *resT
			#print temp.shape
			thisJ = math.sqrt(np.linalg.det(2*math.pi*S)) * math.exp(temp)
			#print (thisJ)

			#update correspondence if the probability is higher than
			#the previous maximum
			if(thisJ > maxJ):
				maxJ = thisJ
				landMarkIndex = j
			
			j = j + 1	
		flag = 0
		
		
		K = covBar*np.transpose(HRec[:,:,landMarkIndex])*np.linalg.inv(SRec[:,:,landMarkIndex])
		if(row>=720):
			print row
		resTemp = np.transpose(zVecRec[row,:]-zVecRec[landMarkIndex, :])
		resTemp = np.reshape(resTemp, (2,1)) 
		
		muBar = muBar + K*resTemp

		row = row + 1
	mu = hMu
	cov = hCov
	
	print mu



def odomCallback(msg):
	global x, y, theta, u, mu, cov, m
    
    #retrieve the pose from odometry
	#x = msg.pose.pose.position.x
	#y = msg.pose.pose.position.y
	#theta = msg.pose.pose.orientation.z

	#retrieve linear and angular velocity
	u = np.matrix([
		[msg.twist.twist.linear.x],
		[msg.twist.twist.angular.z]])

	


def laserCallback(msg):
	global z
	#retrieve the laser range data
	z = msg.ranges
	ekf_localization(mu, cov, u, z, m);


def main():
	print("Husky_ekf_localization start!!!")

	rospy.init_node('Husky_ekf_localization',anonymous = True)
	rospy.Subscriber("/husky_velocity_controller/odom",Odometry,odomCallback)
	rospy.Subscriber("scan",LaserScan,laserCallback)
	rospy.spin()


if __name__ == '__main__':
	main()
