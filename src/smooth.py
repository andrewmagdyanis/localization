#!/usr/bin/env python
import roslib
import rospy
import math
import time
import numpy as np
import os
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import Int32
from rospy_tutorials.msg import Floats
from gpiozero import PWMOutputDevice
  #initialize all variables
  #current positon
     Xc=0
     Yc=0
#final position
    Xf=0
    Yf=0
#goal angel
   Theta_g=0
#current angel
   Theta_c=0
#initial value of flag
   Flag =0
   R=3.25 #radius of wheel cm
   L=18.5 #seperation distance cm
#define publishers
  pub1 = rospy.Publisher('Wr_target_Rob3', Int32, queue_size=10)
  pub2 = rospy.Publisher('Wl_target_Rob3', Int32, queue_size=10)
  pub3 = rospy.Publisher('Flag_3', Int32, queue_size=10)

  #get data from the callbacks(goal angel, current and final positions)
def callback1(data):
    try:
       global Theta_g
       Theta_g=data.data[0]
   except IndexError:
   pass
def callback2(data):
     global Xc, Yc , Theta_c  #C=current position
     Xc=data.data[0]
     Yc=data.data[1]
     Theta_c=data.data[2]
def callback3(data):
     global Xf, Yf   # f=final position
Xf=data.data[0]
Yf=data.data[1]
#run the smooth function

def callback4(data):
    smooth()
    #set the subscribers
def listener():
    rospy.init_node('SmoothController_Rob3')
    rospy.Subscriber('theta_goal_Rob3' ,Int32MultiArray,callback1)
    rospy.Subscriber('rob3_CurrentPose',Int32MultiArray,callback2)
    rospy.Subscriber('robot3_goal_cm' ,Int32MultiArray,callback3)
    rospy.Subscriber('len_route3' ,Int32,callback4)
def smooth():
# get the error in the global reference frame
    if ((Xf > 0) and (Yf >0)) :
    global Flag
    errorX= Xf - Xc
    errorY= Yf - Yc
    error_th = Theta_c - Theta_g
    error_th_rad = error_th * (math.pi / 180)
    theta_rad= Theta_c * (math.pi / 180)

    #get error in the robot's ref frame
   gr_X=round( (errorX*math.cos(theta_rad))+(errorY*math.sin(theta_rad)),2)
   gr_Y=round( (-errorX*math.sin(theta_rad))+(errorY*math.cos(theta_rad)),2)
#calculate Rho and alpha
   rho =round((math.sqrt(gr_X**2 + gr_Y**2)),2)
   alpha = round(error_th_rad,2)
   if alpha > math.pi: #alpha [ -pi , pi ]
   alpha = alpha - (math.pi*2)
#define gains
   K_rho=0.14
   K_alpha=0.3102
#calculate control commands
   while ((abs(gr_X) <= 4 ) and (abs(gr_Y) <= 4) ):
   print 'Reached The goal'
#if reached goal set angular velocities zero and raise the flag
   WR= 0
   WL =0
   Flag =1
#publish angular velocities and raised flags
  pub1.publish(WR)
  pub2.publish(WL)
  pub3.publish(Flag)
#updating the error
  listener()
  errorX= Xf - Xc
  errorY= Yf - Yc
  error_th = Theta_c - Theta_g
  theta_rad= Theta_c * (math.pi / 180)
  gr_X=round( (errorX*math.cos(theta_rad))+(errorY*math.sin(theta_rad)),2)
  gr_Y=round( (-errorX*math.sin(theta_rad))+(errorY*math.cos(theta_rad)),2)
#reset flag
  Flag =0
  pub3.publish(Flag)
  #calculate linear and angular velocity
  V=round((K_rho *rho),2)
  V=max(min(15,V),1.8)
  W= round((K_alpha *alpha),2)
#kinematics
  WR = round(abs((V + ((W*L)/2)) /R)) #right_wheel_angular_vel
  WL = round(abs((V - ((W*L)/2)) /R)) #left_wheel_angular_vel
  pub1.publish(WR)
  pub2.publish(WL)
  listener()
#print WR ,WL
#
  os.system('clear')
  if __name__ == '__main__':
  while not rospy.is_shutdown():
listener()
rospy.spin()
