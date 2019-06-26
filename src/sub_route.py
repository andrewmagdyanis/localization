#!/usr/bin/env python
import math, time
import roslib
import rospy
from std_msgs.msg import Int32
import os
from std_msgs.msg import Int32MultiArray
#initialize all variables

# robot 1 current_position
xR1=0
yR1=0

#obstacle 1 position
xO1=0
yO1=0

#obstacle 2 position
xO2=0
yO2=0

rob1_pose=[]

#get all data from callbacks(positions of all robots and obstacles)
def callback1(data):
        global xR1,yR1
        xR1=data.data[0]
        yR1=data.data[1]
def callback2(data):
        global xR2,yR2
        xR2=data.data[0]
        yR2=data.data[1]
def callback3(data):
        global xR3,yR3
        xR3=data.data[0]
        yR3=data.data[1]
def callback4(data):
        global xR4,yR4
        xR4=data.data[0]
        yR4=data.data[1]
def callback5(data):
        global xO1,yO1
        xO1=data.data[0]
        yO1=data.data[1]
def callback6(data):
        global xO2,yO2
        xO2=data.data[0]
        yO2=data.data[1]
def callback7(data):
        global rob1_pose
        rob1_pose=data.data
def callback8(data):
        global rob2_pose
        rob2_pose=data.data
def callback9(data):
        global rob3_pose
        rob3_pose=data.data
        map_()
#initialize the node and set the subscribers
def listener():
    rospy.init_node('Tracking')
    rospy.Subscriber('robot1',Int32MultiArray,callback1)
    #rospy.Subscriber('robot2',Int32MultiArray,callback2)
    #rospy.Subscriber('robot3',Int32MultiArray,callback3)
    rospy.Subscriber('obst1',Int32MultiArray,callback4)
    rospy.Subscriber('obst2',Int32MultiArray,callback5)
    rospy.Subscriber('rob1_CurrentPose',Int32MultiArray,callback6)
    #rospy.Subscriber('rob2_CurrentPose',Int32MultiArray,callback7)
    #rospy.Subscriber('rob3_CurrentPose',Int32MultiArray,callback8)
#define a map(2D array of ones and zeros)
def map_():
      n=10
      m=10
      the_map = []
      row = [0] *n
      for i in range(m):
          the_map.append(list(row))
      #set the robots and the obstacles positions on the map
      the_map[0][0] = 6
      the_map[yR1][xR1] = 1
      #the_map[yR2][xR2] = 2
      #the_map[yR3][xR3] = 3
      the_map[yO1][xO1] = 4
      the_map[yO2][xO2] = 4
      print 'Map size (X,Y):', n, m
      print "robot1 on map(x,y):" , xR1 , yR1
      #print "robot2 on map(x,y):" , xR2 , yR2
      #print "robot3 on map(x,y):" , xR3 , yR3
      #print "robot4 on map(x,y):" , xR4 , yR4
      print "obstacle1 on map(x,y):", xO1 , yO1
      print "obstacle2 on map(x,y):", xO2 , yO2
      print 'rob1_current position(x,y,w):' ,rob1_pose
      #print 'rob2_current position(x,y,w):' ,rob2_pose
      #print 'rob3_current position(x,y,w):' ,rob3_pose
      #calculate the time taken to generate the map
      t = time.time()
      print 'Time=', time.time() - t
      # display the map with the route added
      print 'Map:'
      for y in range(m):
             for x in range(n):
                   xy = the_map[y][x]
                   if xy == 0:
                      print '2', # space
                   elif xy == 1:
                      print '1', # robot1
                   #elif xy == 2:
                    #  print '2', # robot2
                   #elif xy == 3:
                    #  print '3', # robot3
                   elif xy == 4:
                      print '-1', # obstacle1 & obstacle2
                   elif xy == 6:
                      print 'R', # refrence

      os.system('clear')

if __name__ == '__main__':
     while not rospy.is_shutdown():
                listener()
                rospy.spin()
