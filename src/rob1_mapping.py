#!usr/bin/env python
import math
import time
import roslib
import rospy
from std_msgs.msg import Int32
from std_msgs.msg import Int32MultiArray
import os
import numpy as np


mapping_rob1= rospy.Publisher('rob1_map', Int32MultiArray, queue_size=10)

# initialize all variables

#robot 1 current position
x_R1 = 0
y_R1 = 0

#obstacle 1 position
x_Ob1 = 0
y_Ob1 = 0

#obstacle 2 position
x_Ob2 = 0
y_Ob2 = 0

rob1_pose = []

def callback1(msg):
    global x_R1,Y_R1
    x_R1= msg.data[0]
    y_R1= msg.data[1]
def callback2(msg):
    global x_Ob1, y_Ob2
    x_Ob1= msg.data[0]
    y_Ob1= msg.data[1]

def callback3(msg):
    global x_Ob2, y_Ob2
    x_Ob2= msg.data[0]
    y_Ob2= msg.data[1]

def callback4(msg):
    global rob1_pose
    rob1_pose= msg.data
    map_()




def listener():
    rospy.init_node('rob1_mapping')

    rospy.Subscriber('robot1', Int32MultiArray, callback1)
    rospy.Subscriber('obst1', Int32MultiArray, callback2)
    rospy.Subscriber('obst2', Int32MultiArray, callback3)
    rospy.Subscriber('rob1_CurrentPose', Int32MultiArray, callback4)

    while not rospy.is_shutdown():

        map1=np.array([xy],Int32MultiArray)

        mapping_rob1.publish(map1)
        print'robot1_map', map1
        rospy.sleep(1)


def map_():
    n=10   # no.of columns
    m=10   # no.of rows
    the_map = []
    row = [0] * n
    for i in range(m):
        the_map.append(list(row))
        # set the positions of robot1 and obstacles on the map
        the_map[0][0]=6
        the_map[y_R1][x_R1]= 1
        the_map[y_Ob1][x_Ob1]= 5
        the_map[y_Ob2][x_Ob2]= 5


        print 'Map size (X,Y):', n,m
        print "robot1 on map(x,y):" , x_R1 , y_R1
        print "obstacle1 on map(x,y):", x_Ob1 , y_Ob1
        print "obstacle2 on map(x,y):", x_Ob2 , y_Ob2
        print 'rob1_current position(x,y,w):' ,rob1_pose

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

                     elif xy == 5:
                        print '-1', # obstacle1 & obstacle2
                     elif xy == 6:
                        print 'R', # refrence

        os.system('clear')

if __name__ == '__main__':
    listener()
