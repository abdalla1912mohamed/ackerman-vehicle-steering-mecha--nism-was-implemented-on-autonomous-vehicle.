#!/usr/bin/env python

#########################################################################################################
#Import the required libraries:
from __future__ import print_function,division
import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Pose,Twist
import numpy as np
import math
import time

time.sleep(10)
#########################################################################################################

#########################################################################################################
#######################################################################
#Initialize ROS Node
rospy.init_node('Autonomous_Systems_MS_4_Planning_Team_37') #Identify ROS Node
#######################################################################

#######################################################################
#ROS Publisher Code for Steering
pub1 = rospy.Publisher('/Longitudinal_Driving_Velocity', Float64, queue_size=10)
pub2 = rospy.Publisher('/Lateral_Distance', Float64, queue_size=10)
rate = rospy.Rate(10) # rate of publishing msg 10hz
#######################################################################

#######################################################################
#ROS Subscriber Variables
Actual_position = Pose()
Actual_Speed = Twist()
flag_St = 0
#######################################################################

#######################################################################
#ROS Subscriber Code for Position and Velocity
def callback_Act_St(data):
 global Actual_position,Actual_Speed	#Identify msg variable created as global variable
 global sub1,flag_St		#Identify a subscriber as global variable
 
 if flag_St == 0:
  Actual_position.position.x = data.position.x
  Actual_position.position.y = data.position.y
  Actual_position.orientation.z = data.orientation.z
  Actual_Speed.linear.x = data.position.z
  flag_St = 1

sub1 = rospy.Subscriber('/Actual_Reading', Pose, callback_Act_St)
#######################################################################
#########################################################################################################

#########################################################################################################
def quaternion_to_euler(x, y, z, w):
 t0 = +2.0 * (w * x + y * z)
 t1 = +1.0 - 2.0 * (x * x + y * y)
 roll = math.atan2(t0, t1)
 t2 = +2.0 * (w * y - z * x)
 t2 = +1.0 if t2 > +1.0 else t2
 t2 = -1.0 if t2 < -1.0 else t2
 pitch = math.asin(t2)
 t3 = +2.0 * (w * z + x * y)
 t4 = +1.0 - 2.0 * (y * y + z * z)
 yaw = math.atan2(t3, t4)
 return [yaw, pitch, roll]
#########################################################################################################

#########################################################################################################
V_Des = 3
Lane_Des = 0.5
#######################################################################
#Simulation While Loop
st_time = time.time()
#######################################################################
while 1 and not rospy.is_shutdown():
 st_time = time.time()
 
 if flag_St == 1:
  V_Act = Actual_Speed.linear.x
 
  Angles_Act = quaternion_to_euler(Actual_position.orientation.x, Actual_position.orientation.y, Actual_position.orientation.z, Actual_position.orientation.w)
  Yaw_act = Angles_Act[0] 
  Pos_Act = [Actual_position.position.x, Actual_position.position.y,Yaw_act]
 
  flag_St = 0  

 pub1.publish(V_Des)	#Publish msg
 pub2.publish(Lane_Des)	#Publish msg
 rate.sleep()		#Sleep with rate
 end_time = time.time()
 tau  = end_time-st_time
#######################################################################
#########################################################################################################




