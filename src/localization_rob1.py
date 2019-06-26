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

#define the publishers

#publish robot1  and obstacles positions in pixels for the map

Robot1 = rospy.Publisher('rob1_pose_px', Int32MultiArray, queue_size=10)
pub = rospy.Publisher('Ob1_pose_px', Int32MultiArray, queue_size=10)
pub2 = rospy.Publisher('Ob2_pose_px', Int32MultiArray, queue_size=10)

#publish the robot1 position for tracking
Robot1_current = rospy.Publisher('rob1_CurrentPose', Int32MultiArray, queue_size=10)

#initialize all variables
x = 0
index = 0
x_ref=0
y_ref=0
X_r1=0
Robot=[0,0,0,0]


def callback(data):
    try:
         global Robot,index,x, x_ref, y_ref ,x_1,y_1
         Robot=[0,0,0,0]
         #get the position and theta for all the apriltags from Id 0 to ID 3
         #put them in one list from the smallest ID to the biggest
         while index <= len(data.detections)-1:
             get_id=data.detections[index].id
             get_x = int((data.detections[index].pose.pose.position.x)*100)
             get_y = int((data.detections[index].pose.pose.position.y)*100)
             get_w = int((data.detections[index].pose.pose.orientation.w)*100)
             get_thetax = int((data.detections[index].pose.pose.orientation.x))
             get_thetay = int((data.detections[index].pose.pose.orientation.y))
             if( get_id == x):
 				Robot[x]= [x,get_x, get_y,get_w, get_thetax , get_thetay ]
 				index = index + 1
 				x = 0
             else :
 				x = x + 1

            index = 0
            x=0



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

         ##the position of obstacle1 w.r.t the camera
         O1_x=Robot[2][1]
         O1_y=Robot[2][2]

         ##the position of obstacle2 w.r.t the camera
         O2_x=Robot[3][1]
         O2_y=Robot[3][2]

         ###the position of the robots w.r.t our Ref
         X_r1_current = x_1 - x_ref
         Y_r1_current = y_1 - y_ref

         O1_x_current = O1_x - x_ref
         O1_y_current = O1_y - y_ref

         O2_x_current = O2_x - x_ref
         O2_y_current = O2_y - y_ref

         ##convert the positions into pixels for the map
         # 1 cm equals to 2.43308 according to our camera calibration
         #our working area equals to 429 pixels (176 cm) on x-axis
         # and 404 pixels (166 cm) on y-axis
         # we divide our area into 100 grids ( 10*10 ) so we divided by 42.9 and 40.4
         #theta equal tan^-1(y/x)
         #we minus 90 since our ref orientation are on 90 degrees not 0

         X_r1= -(-(((X_r1_current) *2.43308))//42.9) - 1 #pixels
         Y_r1= -(-(((Y_r1_current) *2.43308))//40.4) - 1 #pixels
         theta_r1= ((math.degrees(math.atan(thetay_1/thetax_1))*2)-90)

         o1_x= -(-(((O1_x_current) *2.43308))//42.9) - 1 #pixels
         o1_y= -(-(((O1_y_current) *2.43308))//40.4) - 1 #pixels

         o2_x= -(-(((O2_x_current) *2.43308))//42.9) - 1 #pixels
         o2_y= -(-(((O2_y_current) *2.43308))//40.4) - 1 #pixels

         #limit the thetas and the pixels
         if (theta_r1 < 0):
               theta_r1 += 360
         if  X_r1 <= 0:
            X_r1=0
         if X_r1 >= 10:
            X_r1=10
         if Y_r1 <= 0:
            Y_r1=0
         if Y_r1 >= 10:
            Y_r1=10


         #put the results in arrays to publish them

         robot1=np.array([int(X_r1) , int(Y_r1),int(theta_r1)],Int32MultiArray)
         obstacle1=np.array([int(o1_x) , int(o1_y)],Int32MultiArray)
         obstacle2=np.array([int(o2_x) , int(o2_y)],Int32MultiArray)

         robot1_Current=np.array([X_r1_current , Y_r1_current ,int(theta_r1)],Int32MultiArray)

         robot1_xyw= Int32MultiArray(data=robot1)
         obstacle1_xy=Int32MultiArray(data=obstacle1)
         obstacle2_xy=Int32MultiArray(dat=obstacle2)

         robot1_xyw_current= Int32MultiArray(data=robot1_Current)

         Robot1.publish(robot1_xyw)
         pub.publish(obstacle1_xy)
         pub2.publish(obstacle2_xy)

         Robot1_current.publish(robot1_xyw_current)

         print " robot 1 current_pose  = %s" % (robot1_Current)
         print " robot 1 pixles_pose = %s" % (robot1)
         print " obstacle 1 pixles_pose = %s" % (obstacle1)
         print " obstacle 2 pixles_pose = %s" % (obstacle2)

         os.system('clear')





    except TypeError:
           print 'not all tags detected'
           pass

def listener():
    rospy.init_node('Localization_robot1')
    rospy.Subscriber('tag_detections',AprilTagDetectionArray,callback)

if __name__ == '__main__':

	while not rospy.is_shutdown():
		listener()
		rospy.spin()
