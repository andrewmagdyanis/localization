#!/usr/bin/env python
import math, time
import roslib
import rospy
from std_msgs.msg import Int32
import os
import numpy as np
from std_msgs.msg import Int32MultiArray
#initialize all variables

mapping_rob1= rospy.Publisher('rob1_map', Int32MultiArray, queue_size=10)

#initialize all variables


# robot 1 current_position
xR1=0
yR1=0

#robot 2 current_position
xR2=0
yR2=0

#robot 3 current_position
xR3=0
yR3=0

#robot 4 current_pposition
xR4=0
yR4=0

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
def callback10(data):
        global rob4_pose
        rob4_pose=data.data
        #global mapping_r1
        map_()



#initialize the node and set the subscribers
def listener():
    #global the_map
    the_map = Int32MultiArray()
    rospy.init_node('Tracking')
    rospy.Subscriber('robot1',Int32MultiArray,callback1)
    rospy.Subscriber('robot2',Int32MultiArray,callback2)
    rospy.Subscriber('robot3',Int32MultiArray,callback3)
    rospy.Subscriber('robot4',Int32MultiArray,callback4)
    rospy.Subscriber('obst1',Int32MultiArray,callback5)
    rospy.Subscriber('obst2',Int32MultiArray,callback6)
    rospy.Subscriber('rob1_CurrentPose',Int32MultiArray,callback7)
    rospy.Subscriber('rob2_CurrentPose',Int32MultiArray,callback8)
    rospy.Subscriber('rob3_CurrentPose',Int32MultiArray,callback9)
    rospy.Subscriber('rob4_CurrentPose',Int32MultiArray,callback10)

    while not rospy.is_shutdown():
        rospy.sleep(1)



#define a map(2D array of ones and zeros)

def map_():
      #global mapping_r1
      printable_map = [] #The List of Lists
      n=10  #no.of columns
      m=10  #no.of rows
      #global the_map
      the_map = []
      row = [0] *n
      for i in range(m):
          the_map.append(list(row))
      #set the robots and the obstacles positions on the map
      the_map[0][0] = 4
      the_map[yR1][xR1] = 1
      the_map[yR2][xR2] = 4
      the_map[yR3][xR3] = 4
      the_map[yR4][xR4] = 4
      the_map[yO1][xO1] = 4
      the_map[yO2][xO2] = 4



      print 'Map size (X,Y):', n, m
      print "robot1 on map(x,y):" , xR1 , yR1
      print "robot2 on map(x,y):" , xR2 , yR2
      print "robot3 on map(x,y):" , xR3 , yR3
      print "robot4 on map(x,y):" , xR4 , yR4
      print "obstacle1 on map(x,y):", xO1 , yO1
      print "obstacle2 on map(x,y):", xO2 , yO2
      print 'rob1_current position(x,y,w):' ,rob1_pose
      print 'rob2_current position(x,y,w):' ,rob2_pose
      print 'rob3_current position(x,y,w):' ,rob3_pose
      #calculate the time taken to generate the map
      t = time.time()
      print 'Time=', time.time() - t
      # display the map with the route added
      print 'Map:'
      for y in range(m):
             list1= [] #Temp list that is appeneded
             for x in range(n):
                   xy = the_map[y][x]
                   if xy == 0:
                      #print '2', # space
                      list1.append(2)
                   elif xy == 1:
                      #print '1', # robot1
                      list1.append(1)
                   elif xy == 4:
                      #print '-1', # ref,obstacles & the rest of robots
                      list1.append(-1)

                      #print 'the map',the_map
             printable_map.append(list1)
      print printable_map
      mapping_rob1.publish(printable_map)
      os.system('clear')

if __name__ == '__main__':

        listener()
