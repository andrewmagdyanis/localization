#!/usr/bin/env python

import roslib
import rospy
from apriltags_ros.msg import AprilTagDetectionArray
from std_msgs.msg import Int32MultiArray,MultiArrayDimension
import math
import numpy as np
import os
#from nav_msgs.msg import Odometry
#from tf2_msgs.msg import TFMessage
#import tf
#import time

grid_dim=17.5 #cm

#comment for trying 
#define the publishers
################################################################################
#publish the robots and obstacles positions in pixels for the map:
Robot1 = rospy.Publisher('robot1', Int32MultiArray, queue_size=10)
Robot2 = rospy.Publisher('robot2', Int32MultiArray, queue_size=10)
Robot3 = rospy.Publisher('robot3', Int32MultiArray, queue_size=10)
Robot4 = rospy.Publisher('robot4', Int32MultiArray, queue_size=10)
obstacle1 = rospy.Publisher('obst1', Int32MultiArray, queue_size=10)
obstacle2 = rospy.Publisher('obst2', Int32MultiArray, queue_size=10)
#publish the robots positions for tracking in cm:
Robot1_current = rospy.Publisher('rob1_CurrentPose', Int32MultiArray, queue_size=10)
Robot2_current = rospy.Publisher('rob2_CurrentPose', Int32MultiArray, queue_size=10)
Robot3_current = rospy.Publisher('rob3_CurrentPose', Int32MultiArray, queue_size=10)
Robot4_Current = rospy.Publisher('rob4_CurrentPose', Int32MultiArray, queue_size=10)
#publish all robot poses in single array:
pub_robots_current_poses = rospy.publisher('robots_current_poses',Int32MultiArray,queue_size = 10)
x = 0
index = 0
x_ref=0
y_ref=0
X_r1=0

Robot = Int32MultiArray()
get_quaternion_x = get_quaternion_y = get_quaternion_z = get_quaternion_w = 0

def callback(data):
	try:
		global Robot,index,x, x_ref, y_ref ,x_1,y_1,x_2,y_2,get_quaternion_x, get_quaternion_y, get_quaternion_z, get_quaternion_w
		Robot=[0,0,0,0,0,0,0,0]
		while index <= len(data.detections)-1:	#Habda 3anefa mn elsana ely fatet
			get_id=	data.detections[index].id
			get_x = int((data.detections[index].pose.pose.position.x)*100)
			get_y = int((data.detections[index].pose.pose.position.y)*100)
			#get_w = int((data.detections[index].pose.pose.orientation.w)) #Joe byhbed
			get_quaternion_x = data.detections[index].pose.pose.orientation.x
			get_quaternion_y = data.detections[index].pose.pose.orientation.y
			get_quaternion_z = data.detections[index].pose.pose.orientation.z
			get_quaternion_w = data.detections[index].pose.pose.orientation.w
			####################################################################
			explicit_quat = [get_quaternion_x,get_quaternion_y,get_quaternion_z,get_quaternion_w]
			euler = tf.transformations.euler_from_quaternion(explicit_quat)
			get_w = int(euler[2]*180/math.pi)
			####################################################################
			#Looping to assign data acquired  to the right robot
			if( get_id == x):
				Robot[x]= [get_x, get_y,get_w]
				index = index + 1
				x = 0
			else :
				x = x + 1

		index = 0
		x = 0
        #the position of the reference
		x_ref=Robot[0][0]
		y_ref=Robot[0][1]
		w_ref=Robot[0][2]
        #the position of robot 1 w.r.t the camera
		x_1=Robot[1][0]
		y_1=Robot[1][1]
		w_1=Robot[1][2]
        #the position of robot 2 w.r.t the camera
		x_2=Robot[2][0]
		y_2=Robot[2][1]
		w_2=Robot[2][2]
        #the position of robot 3 w.r.t the camera
		x_3=Robot[3][0]
		y_3=Robot[3][1]
		w_3=Robot[3][2]
        #the position of robot 4 w.r.t the camera
		x_4=Robot[4][0]
		y_4=Robot[4][1]
		w_4=Robot[4][2]
		#the position of obstacle 1 w.r.t the camera
		Ob1_x = Robot[5][0]
		Ob1_y = Robot[5][1]
		#the position of obstacle 2 w.r.t the camera
		Ob2_x=Robot[6][0]
		Ob2_y=Robot[6][1]

		#Robot_1:
		########################################################################
		X_r1_current= (x_1 - x_ref) # x-position of robot 1 wrt ref /current position in cm
		Y_r1_current= (y_1 - y_ref) # y-position of robot 1 wrt ref
		#D_r1= math.sqrt((X_r1_current - 0)*(X_r1_current - 0) + (Y_r1_current - 0)*(Y_r1_current - 0)) #distance between robot1 and ref
		#convert the positions into pixels for the map
		X_r1 = (X_r1_current) /grid_dim #pixels
		Y_r1 = ((Y_r1_current) /grid_dim) +1  #pixels
		if X_r1 <= 0:X_r1=0
		if X_r1 >= 9:X_r1=9
		if Y_r1 <= 0:Y_r1=0
		if Y_r1 >= 9:Y_r1=9
		# range of theta from (0 to 180) and from (-180 to 0 )
		W_1= (w_1 - w_ref ) # theta 1 in degrees
		W_r1= 100*((W_1)*math.pi/180)  # theta in radians
		#Robot_2:
		########################################################################
		X_r2_current= (x_2 - x_ref) # x-position of robot 2 wrt ref
		Y_r2_current= (y_2 - y_ref) # y-position of robot 2 wrt ref
		X_r2 = (X_r2_current) /grid_dim  #pixels
		Y_r2 = ((Y_r2_current) /grid_dim) +1  #pixels
		if X_r2	<=0:X_r2=0
		if X_r2 >=9:X_r2=9
		if Y_r2 <=0:Y_r2= 0
		if Y_r2 >9:Y_r2 =9
		#D_r2= math.sqrt( (X_r2 - 0)*(X_r2 - 0) + (Y_r2 - 0)*(Y_r2 - 0)) #distance between robot2 and ref
		W_2= (w_2 - w_ref ) # theta 2 in degrees
		W_r2= 100*((W_2)*math.pi/180)  # theta 2 in radians
        #Robot_3:
		########################################################################
		X_r3_current= (x_3 - x_ref) # x-position of robot 3 wrt ref
		Y_r3_current= (y_3 - y_ref) # y-position of robot 3 wrt ref
		X_r3 = (X_r3_current) /grid_dim  #pixels
		Y_r3 = ((Y_r3_current) /grid_dim) +1  #pixels
		if X_r3 <=0:X_r3=0
		if Y_r3 <=0:Y_r3= 0
		if Y_r3 >=9:Y_r3 =9
		if X_r3 >=9:X_r3=9
		#D_r3= math.sqrt( (X_r3 - 0)*(X_r3 - 0) + (Y_r3 - 0)*(Y_r3 - 0)) #distance between robot3 and ref
		W_3= (w_3 - w_ref ) # theta 3 in degrees
		W_r3= 100*((W_3)*math.pi/180)  # theta in radians
		#Robot_4:
		########################################################################
		X_r4_current= (x_4 - x_ref) # x-position of robot 4 wrt ref
		Y_r4_current= (y_4 - y_ref) # y-position of robot 4 wrt ref
		X_r4 = (X_r4_current) /grid_dim  #pixels
		Y_r4 = ((Y_r4_current) /grid_dim) +1 #pixels
		if X_r4 <=0:X_r4=0
		if X_r4>=9:X_r4=9
		if Y_r4 <=0:Y_r4= 0
		if Y_r4 >=9:Y_r4 =9
		#D_r4= math.sqrt( (X_r4 - 0)*(X_r4 - 0) + (Y_r4 - 0)*(Y_r4 - 0)) #distance between robot4 and ref
		W_4= (w_4 - w_ref ) # theta 1 in degrees
		W_r4= 100*((W_4)*math.pi/180)  # theta in radians
		#Obstacle_1:
		########################################################################
		Ob1_x_current= (Ob1_x - x_ref) # x-position of obstacle 1 wrt ref
		Ob1_y_current= (Ob1_y - y_ref) # y-position of obstacle 1 wrt
		ob1_x = (Ob1_x_current) /grid_dim  #pixels
		ob1_y = ((Ob1_y_current)/grid_dim) +1  #pixels
		#Obstacle_2:
		########################################################################
		Ob2_x_current= (Ob2_x - x_ref) #x-position of obstacle 2 wrt ref
		Ob2_y_current= (Ob2_y - y_ref) #y-position of obstacle 2 wrt ref
		ob2_x = (Ob2_x_current) /grid_dim  #pixels
		ob2_y = ((Ob2_y_current) /grid_dim) +1  #pixels
		#put the results in arrays and publish them

		robot1_xyw_current= Int32MultiArray(data=np.array([X_r1_current, Y_r1_current, W_r1],Int32MultiArray))
		robot2_xyw_current= Int32MultiArray(data=np.array([X_r2_current, Y_r2_current, W_r2],Int32MultiArray))
		robot3_xyw_current= Int32MultiArray(data=np.array([X_r3_current, Y_r3_current, W_r3],Int32MultiArray))
		robot4_xyw_current= Int32MultiArray(data=np.array([X_r4_current, Y_r4_current, W_r4],Int32MultiArray))

		robot1_xyw= Int32MultiArray(data=np.array([int(X_r1) , int(Y_r1) , (W_r1)],Int32MultiArray))
		robot2_xyw= Int32MultiArray(data=np.array([int(X_r2) , int(Y_r2) , (W_r2)],Int32MultiArray))
		robot3_xyw= Int32MultiArray(data=np.array([int(X_r3) , int(Y_r3) , (W_r3)] , Int32MultiArray))
		robot4_xyw= Int32MultiArray(data=np.array([int(X_r4) , int(Y_r4) , (W_r4)] , Int32MultiArray))
		obst1_xy= Int32MultiArray(data=np.array([int(ob1_x) , int(ob1_y)] , Int32MultiArray))
		obst2_xy= Int32MultiArray(data=np.array([int(ob2_x) , int(ob2_y)] , Int32MultiArray))

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

		pub_robots_current_poses.publish(Int32MultiArray(data=[
		X_r1_current, Y_r1_current,
		X_r2_current, Y_r2_current,
		X_r3_current, Y_r3_current,
		X_r4_current, Y_r4_current],Int32MultiArray))

		#print "robot 1 thetaaaa in degrees = %s " % (W_1)
		#print "robot 1 wrt to ref in cm = %s " %  (robot1_Current)
		#print "robot 2 wrt to ref in cm = %s" %  (robot2_Current)
		#print "robot 3 wrt to ref in cm = %s" %  (robot3_Current)
		#print "robot 4 wrt to ref in cm = %s" %  (robot4_Current)
		#print "robot1_pose  in pixles = %s" % (robot1)
		#print "robot2_pose in pixels = %s" % (robot2)
		#print "robot3_pose in pixles = %s" % (robot3)
		#print "robot4_pose in pixels = %s" % (robot4)
		#print "obstacle1_pose in pixels = %s" % (obst1)
		#print "obstacle2_pose in pixles = %s" % (obst2)

		os.system('clear')
	except rospy.ROSInterruptException:
		print 'not all tags detected'
		pass

def listener():
	rospy.init_node('localization')
	rospy.Subscriber('tag_detections',AprilTagDetectionArray,callback)

if __name__ == '__main__':
	while not rospy.is_shutdown():
		listener()
		rospy.spin()
