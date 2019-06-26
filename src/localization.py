#!/usr/bin/env python

from __future__ import division
import roslib
import rospy
from std_msgs.msg import Int32
from apriltags_ros.msg import AprilTagDetectionArray
from std_msgs.msg import Int32MultiArray
import math
import time
import numpy as np
import os

#define the publishers

#publish the robots and obstacles positions in pixels for the map

Robot1 = rospy.Publisher('robot1', Int32MultiArray, queue_size=10)
Robot2 = rospy.Publisher('robot2', Int32MultiArray, queue_size=10)
Robot3 = rospy.Publisher('robot3', Int32MultiArray, queue_size=10)
Robot4 = rospy.Publisher('robot4', Int32MultiArray, queue_size=10)
obstacle1 = rospy.Publisher('obst1', Int32MultiArray, queue_size=10)
obstacle2 = rospy.Publisher('obst2', Int32MultiArray, queue_size=10)


#publish the robots positions for tracking
Robot1_current = rospy.Publisher('rob1_CurrentPose', Int32MultiArray, queue_size=10)
Robot2_current = rospy.Publisher('rob2_CurrentPose', Int32MultiArray, queue_size=10)
Robot3_current = rospy.Publisher('rob3_CurrentPose', Int32MultiArray, queue_size=10)
Robot4_Current = rospy.Publisher('rob4_CurrentPose', Int32MultiArray, queue_size=10)
#rate=rospy.Rate(20)

x = 0
index = 0
x_ref=0
y_ref=0
X_r1=0
Robot=[0,0,0,0,0,0,0]


def callback(data):
	try:
		global Robot,index,x, x_ref, y_ref ,x_1,y_1,x_2,y_2,thetax_ref,thetay_ref,thetaz_ref,thetax_1,thetay_1,thetaz_1 #,thetax_2,thetay_2
		Robot=[0,0,0,0,0,0,0]
		while index <= len(data.detections)-1:
			get_id=	data.detections[index].id
			get_x = int((data.detections[index].pose.pose.position.x)*100)
			get_y = int((data.detections[index].pose.pose.position.y)*100)
			get_w = int((data.detections[index].pose.pose.orientation.w)*100)
			get_thetax = int((data.detections[index].pose.pose.orientation.x))
			get_thetay = int((data.detections[index].pose.pose.orientation.y))
			get_thetaz = int((data.detections[index].pose.pose.orientation.z))
			if( get_id == x):
				Robot[x]= [x,get_x, get_y,get_w, get_thetax , get_thetay , get_thetaz ]
				index = index + 1
				x = 0
			else :
				x = x + 1

		index = 0
		x = 0
        #the position of the reference

		x_ref=Robot[0][1]
		y_ref=Robot[0][2]
		thetax_ref=Robot[0][3]
		thetay_ref=Robot[0][4]


        #the position of robot 1 w.r.t the camera

		x_1=Robot[1][1]
		y_1=Robot[1][2]
		thetax_1=Robot[1][3]
		thetay_1=Robot[1][4]
		thetaz_1=Robot[1][5]


        #the position of robot 2 w.r.t the camera

		x_2=Robot[2][1]
		y_2=Robot[2][2]
		thetax_2=Robot[2][3]
		thetay_2=Robot[2][4]


        #the position of robot 3 w.r.t the camera

		x_3=Robot[3][1]
		y_3=Robot[3][2]
		thetax_3=Robot[3][3]
		thetay_3=Robot[3][4]
		#w_3=Robot[3][3]

        #the position of robot 4 w.r.t the camera

		x_4=Robot[4][1]
		y_4=Robot[4][2]
		thetax_4=Robot[4][3]
		thetay_4=Robot[4][4]


		#the position of obstacle1 w.r.t the camera
		Ob1_x = Robot[5][1]
		Ob1_y = Robot[5][2]

		#the position of obstacle2 w.r.t the camera

		Ob2_x=Robot[6][1]
		Ob2_y=Robot[6][2]

		#convert the positions into pixels for the map

		### Robot 1 ###

		#X_r1= -(-(((x_1 - x_ref) *3.6)+36)//36) - 2 #pixels
		#Y_r1= -(-(((y_1 - y_ref) *3.6)+36)//36) - 2 #pixels

		X_r1_current= (x_1 - x_ref) # x-position of robot 1 wrt ref /current position in cm
		Y_r1_current= (y_1 - y_ref) # y-position of robot 1 wrt ref

		D_r1= math.sqrt((X_r1_current - 0)*(X_r1_current - 0) + (Y_r1_current - 0)*(Y_r1_current - 0)) #distance between robot1 and ref
		X_r1= -(-(((X_r1_current) *2.43308))//42.9) - 1 #pixels
		Y_r1= -(-(((Y_r1_current) *2.43308))//40.4) - 1 #pixels


		theta_r1=(math.degrees(math.atan(thetay_1/thetax_1))*2)

		### Robot 2 ###

		#X_r2= -(-(((x_2 - x_ref) *3.6)+36)//36) - 2 #pixels
		#Y_r2= -(-(((y_2 - y_ref) *3.6)+36)//36) - 2 #pixels

		X_r2_current= (x_2 - x_ref) # x-position of robot 2 wrt ref
		Y_r2_current= (y_2 - y_ref) # y-position of robot 2 wrt ref

		X_r2=-(-(((X_r2_current) *2.43308))//42.9) - 1 #pixels
		Y_r2=-(-(((Y_r2_current) *2.43308))//40.4) - 1 #pixels

		D_r2= math.sqrt( (X_r2 - 0)*(X_r2 - 0) + (Y_r2 - 0)*(Y_r2 - 0)) #distance between robot2 and ref

		theta_r2= (math.degrees(math.atan(thetay_2/thetax_2))*2)


        ### Robot(3) ###


		X_r3_current= (x_3 - x_ref) # x-position of robot 3 wrt ref
		Y_r3_current= (y_3 - y_ref) # y-position of robot 3 wrt ref

		X_r3=-(-(((X_r3_current) *2.43308))//42.9) - 1 #pixels
		Y_r3=-(-(((Y_r3_current) *2.43308))//40.4) - 1 #pixels

		D_r3= math.sqrt( (X_r3 - 0)*(X_r3 - 0) + (Y_r3 - 0)*(Y_r3 - 0)) #distance between robot3 and ref

		theta_r3=(math.degrees(math.atan(thetay_3/thetax_3))*2)


		### Robot(4) ###


		X_r4_current= (x_4 - x_ref) # x-position of robot 4 wrt ref
		Y_r4_current= (y_4 - y_ref) # y-position of robot 4 wrt ref

		X_r4=-(-(((X_r4_current) *2.43308))//42.9) - 1 #pixels
		Y_r4=-(-(((Y_r4_current) *2.43308))//40.4) - 1 #pixels

		D_r4= math.sqrt( (X_r4 - 0)*(X_r4 - 0) + (Y_r4 - 0)*(Y_r4 - 0)) #distance between robot4 and ref

		theta_r4=(math.degrees(math.atan(thetay_4/thetax_4))*2)



		### Obstacle(1) ###

		Ob1_x_current= (Ob1_x - x_ref) # x-position of obstacle 1 wrt ref
		Ob1_y_current= (Ob1_y - y_ref) # y-position of obstacle 1 wrt

		ob1_x=-(-(((Ob1_x_current) *2.43308))//42.9) - 1 #pixels
		ob1_y=-(-(((Ob1_y_current) *2.43308))//40.4) - 1 #pixels

		#ob1_x = -(-(((Ob1_x_current) *3.6)+36)//36) - 2 #pixels
		#ob1_y = -(-(((Ob1_y_current) *3.6)+36)//36) - 2 #pixels


		### Obstacle(2) ###

		Ob2_x_current= (Ob2_x - x_ref) #x-position of obstacle 2 wrt ref
		Ob2_y_current= (Ob2_y - y_ref) #y-position of obstacle 2 wrt ref

		ob2_x=-(-(((Ob2_x_current) *2.43308))//42.9) - 1 #pixels
		ob2_y=-(-(((Ob2_y_current) *2.43308))//40.4) - 1 #pixels

		#ob2_x = -(-(((Ob2_x_current) *3.6)+36)//36) - 2 #pixels
		#ob2_y = -(-(((Ob2_y_current) *3.6)+36)//36) - 2 #pixels



		# limit the thetas and the pixels

		if (theta_r1 < 0):
		         theta_r1 += 360

		if X_r1 <= 0:
		      X_r1=0
		if X_r1 >= 10:
		      X_r1=10
		if Y_r1 <= 0:
		      Y_r1=0
		if Y_r1 >= 10:
		      Y_r1=10

		if (theta_r2 < 0):
			     theta_r2 += 360

		if X_r2	<=0:
			X_r2=0

		if X_r2 >=10:
			X_r2=10

		if Y_r2 <=0:
			Y_r2= 0

		if Y_r2 >=10:
			Y_r2 =10

		if (theta_r3 < 0):
			     theta_r3 += 360

		if X_r3 <=0:
			X_r3=0

		if Y_r3 <=0:
			Y_r3= 0

		if Y_r3 >=10:
			Y_r3 =10

		if X_r3 >=10:
			X_r3=10

		if (theta_r4 < 0):
			     theta_r4 += 360

		if X_r4 <=0:
			X_r4=0

		if X_r4>=10:
			X_r4=10

		if Y_r4 <=0:
			Y_r4= 0
		if Y_r4 >=10:
			Y_r4 =10

		#put the results in arrays to publish them

		robot1_Current=np.array([X_r1_current, Y_r1_current, theta_r1],Int32MultiArray)
		robot2_Current=np.array([X_r2_current, Y_r2_current, theta_r2],Int32MultiArray)
		robot3_Current=np.array([X_r3_current, Y_r3_current, theta_r3],Int32MultiArray)
		robot4_Current=np.array([X_r4_current, Y_r4_current, theta_r4],Int32MultiArray)


		robot1=np.array([int(X_r1) , int(Y_r1) , int(theta_r1)],Int32MultiArray)
		robot2=np.array([int(X_r2) , int(Y_r2) , int(theta_r2)],Int32MultiArray)
		robot3=np.array([int(X_r3) , int(Y_r3) , int(theta_r3)] , Int32MultiArray)
		robot4=np.array([int(X_r4) , int(Y_r4) , int(theta_r4)] , Int32MultiArray)
		obst1=np.array([int(ob1_x) , int(ob1_y)] , Int32MultiArray)
		obst2=np.array([int(ob2_x) , int(ob2_y)] , Int32MultiArray)




		robot1_xyw_current= Int32MultiArray(data=robot1_Current)
		robot2_xyw_current= Int32MultiArray(data=robot2_Current)
		robot3_xyw_current= Int32MultiArray(data=robot3_Current)
		robot4_xyw_current= Int32MultiArray(data=robot4_Current)


		robot1_xyw= Int32MultiArray(data=robot1)
		robot2_xyw= Int32MultiArray(data=robot2)
		robot3_xyw= Int32MultiArray(data=robot3)
		robot4_xyw= Int32MultiArray(data=robot4)
		obst1_xy= Int32MultiArray(data=obst1)
		obst2_xy= Int32MultiArray(data=obst2)



		Robot1.publish(robot1_xyw)
		Robot2.publish(robot2_xyw)
		Robot3.publish(robot3_xyw)
		Robot4.publish(robot4_xyw)
		obstacle1.publish(obst1_xy)
		obstacle2.publish(obst2_xy)


		Robot1_current.publish(robot1_xyw_current)
		Robot2_current.publish(robot2_xyw_current)
		Robot3_current.publish(robot3_xyw_current)
		Robot4_Current.publish(robot4_xyw_current)




		print "thetax_1 = %s " % (thetax_1)
		print "thetay_1 = %s " % (thetay_1)
		print "thetaz_1 = %s " % (thetaz_1)
		print "robot 1 wrt to ref in cm = %s " %  (robot1_Current)
		print "robot 2 wrt to ref in cm = %s" %  (robot2_Current)
		print "robot 3 wrt to ref in cm = %s" %  (robot3_Current)
		print "robot 4 wrt to ref in cm = %s" %  (robot4_Current)
		print "robot1_pose  in pixles = %s" % (robot1)
		print "robot2_pose in pixels = %s" % (robot2)
		print "robot3_pose in pixles = %s" % (robot3)
		print "robot4_pose in pixels = %s" % (robot4)
		print "obstacle1_pose in pixels = %s" % (obst1)
		print "obstacle2_pose in pixles = %s" % (obst2)
		print "thetax_1 = %s " % (thetax_1)


		os.system('clear')

	except TypeError:
		print 'not all tags detected'
		pass

def listener():
	rospy.init_node('localization')
	rospy.Subscriber('tag_detections',AprilTagDetectionArray,callback)


if __name__ == '__main__':
	#rate=rospy.Rate(20)
	while not rospy.is_shutdown():
		listener()
	#	rate.sleep()
		rospy.spin()
