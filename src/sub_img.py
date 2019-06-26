#!/usr/bin/env python

import roslib
import rospy
from std_msgs.msg import Int32
from apriltags_ros.msg import AprilTagDetectionArray
from std_msgs.msg import Int32MultiArray
import math
import time
import numpy as np
import os

Robot1 = rospy.Publisher('robot1', Int32MultiArray, queue_size=10)
Robot2 = rospy.Publisher('robot2', Int32MultiArray, queue_size=10)
Robot3 = rospy.Publisher('robot3', Int32MultiArray, queue_size=10)
Robot4 = rospy.Publisher('robot4', Int32MultiArray, queue_size=10)

x = 0
index = 0
x_ref=0
y_ref=0
X_r1=0
Robot=[0,0,0,0,0,0,0]


def callback(data):
	try:
		global Robot,index,x, x_ref, y_ref ,x_1,y_1,x_2,y_2
		Robot=[0,0,0,0,0,0,0]
		while index <= len(data.detections)-1:
			get_id=	data.detections[index].id
			get_x = int((data.detections[index].pose.pose.position.x)*100)
			get_y = int((data.detections[index].pose.pose.position.y)*100)
			#get_w = int((data.detections[index].pose.pose.orientation.w)*100)
			get_thetax = int((data.detections[index].pose.pose.orientation.x))
			get_thetay = int((data.detections[index].pose.pose.orientation.y))
			if( get_id == x):
				Robot[x]= [x,get_x, get_y, get_thetax , get_thetay ]
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
		#w_ref=Robot[0][3]

        #the position of robot 1 w.r.t the camera
		x_1=Robot[1][1]
		y_1=Robot[1][2]
		thetax_1=Robot[1][3]
		thetay_1=Robot[1][4]
		#w_1=Robot[1][3]

        #the position of robot 2 w.r.t the camera
		x_2=Robot[2][1]
		y_2=Robot[2][2]
		#thetax_2=Robot[2][3]
		#thetay_2=Robot[2][4]
		#w_2=Robot[2][3]

        #the position of robot 3 w.r.t the camera
		x_3=Robot[3][1]
		y_3=Robot[3][2]
		#thetax_3=Robot[3][3]
		#thetay_3=Robot[3][4]
		#w_3=Robot[3][3]

        #the position of robot 4 w.r.t the camera
		x_4=Robot[4][1]
		y_4=Robot[4][2]
		#thetax_4=Robot[4][3]
		#thetay_4=Robot[4][4]

        #the position of obstacle1 w.r.t the camera
        #Ob1_x=Robot[5][1]
        #Ob1_y=Robot[5][2]
        #the position of obstacle2 w.r.t the camera
        #Ob2_x=Robot[6][1]
        #Ob2_y=Robot[6][2]


		#X_r1= -(-(((x_1 - x_ref) *3.6)+36)//36) - 2 #pixels
		#Y_r1= -(-(((y_1 - y_ref) *3.6)+36)//36) - 2 #pixels
		X_r1_current= (x_1 - x_ref) # x-position of robot 1 wrt ref /current position in cm
		Y_r1_current= (y_1 - y_ref) # y-position of robot 1 wrt ref
		D_r1_current= math.sqrt((X_r1_current - 0)*(X_r1_current - 0) + (Y_r1_current - 0)*(Y_r1_current - 0)) #distance between robot1 and ref
		#W_r1= w_1 - w_ref

		#X_r1= -(-(((X_r1_current) *2.43308))//42.9) - 1 #pixels
		#Y_r1= -(-(((Y_r1_current) *2.43308))//40.4) - 1 #pixels
		#theta_r1=((math.degrees(math.atan(thetay_1/thetax_1))*2)-90)

		X_r2= -(-(((x_2 - x_ref) *3.6)+36)//36) - 2 #pixels
		Y_r2= -(-(((y_2 - y_ref) *3.6)+36)//36) - 2 #pixels
		#X_r2= (x_2 - x_ref) # x-position of robot 2 wrt ref
		#Y_r2= (y_2 - y_ref) # y-position of robot 2 wrt ref
		D_r2= math.sqrt( (X_r2 - 0)*(X_r2 - 0) + (Y_r2 - 0)*(Y_r2 - 0)) #distance between robot2 and ref
		#W_r2= w_2 - w_ref


		X_r3= -(-(((x_3 - x_ref) *3.6)+36)//36) - 2 #pixels
		Y_r3= -(-(((y_3 - y_ref) *3.6)+36)//36) - 2 #pixels
		#X_r3= (x_3 - x_ref) # x-position of robot 3 wrt ref
		#Y_r3= (y_3 - y_ref) # y-position of robot 3 wrt ref
		D_r3= math.sqrt( (X_r3 - 0)*(X_r3 - 0) + (Y_r3 - 0)*(Y_r3 - 0)) #distance between robot3 and ref
		#W_r3= w_3 - w_ref

		X_r4= -(-(((x_4 - x_ref) *3.6)+36)//36) - 2 #pixels
		Y_r4= -(-(((y_4 - y_ref) *3.6)+36)//36) - 2 #pixels
		#X_r4= (x_4 - x_ref) # x-position of robot 4 wrt ref
		#Y_r4= (y_4 - y_ref) # y-position of robot 4 wrt ref
		D_r4= math.sqrt( (X_r4 - 0)*(X_r4 - 0) + (Y_r4 - 0)*(Y_r4 - 0)) #distance between robot4 and ref
		#W_r3= w_3 - w_ref

		#if (theta_r1 < 0):
		#theta_r1 =  theta_r1 + 360

		#if X_r1 <= 0:
		#X_r1=0
		#if X_r1 >= 10:
		#X_r1=10
		#if Y_r1 <= 0:
		#Y_r1=0
		#if Y_r1 >= 10:
		#Y_r1=10

		robot1=np.array([X_r1_current ,Y_r1_current ,D_r1_current],Int32MultiArray)
		robot2=np.array([X_r2 , Y_r2 , D_r2],Int32MultiArray)
		robot3=np.array([X_r3 , Y_r3 , D_r3],Int32MultiArray)
		robot4=np.array([X_r4 , Y_r4 , D_r4],Int32MultiArray)

		robot1_xyw= Int32MultiArray(data=robot1)
		robot2_xyw= Int32MultiArray(data=robot2)
		robot3_xyw= Int32MultiArray(data=robot3)
		robot4_xyw= Int32MultiArray(data=robot3)

		Robot1.publish(robot1_xyw)
		Robot2.publish(robot2_xyw)
		Robot3.publish(robot3_xyw)
		Robot4.publish(robot4_xyw)

		print Robot
		print "robot1 wrt to ref = %s" %  (robot1)
		print "robot2 wrt to ref = %s" %  (robot2)
		print "robot3 wrt to ref = %s" %  (robot3)
		print "robot4 wrt to ref = %s" %  (robot4)

		os.system('clear')

	except TypeError:
		print 'not all tags detected'
		pass

def listener():
	rospy.init_node('test')
	rospy.Subscriber('tag_detections',AprilTagDetectionArray,callback)


if __name__ == '__main__':

	while not rospy.is_shutdown():
		listener()
		rospy.spin()
