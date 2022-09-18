#!/usr/bin/env python

#########################################################################################################
#Import the required libraries:
from __future__ import print_function,division
import rospy
from std_msgs.msg import Float64
import numpy as np
import math
import time
import serial

time.sleep(10)
#########################################################################################################

#########################################################################################################
#######################################################################
#Initialize ROS Node
rospy.init_node('Autonomous_Systems_MS_3_CLR_Control_37') #Identify ROS Node
Message = serial.Serial('/dev/ttyACM1', 9600)
#######################################################################

#######################################################################
#ROS Publisher Code for Steering
rate = rospy.Rate(10) # rate of publishing msg 10hz
#######################################################################

#######################################################################
#ROS Suscribers Variables
flag_Steer = 0
flag_Vel = 0
Steer_Cont = 0
Vel_Cont = 0
#######################################################################

#######################################################################
#ROS Subscriber Code for Steering
def callback_Control_Steering(data):
 global Steer_Cont	#Identify msg variable created as global variable
 global sub1,flag_Steer		#Identify a subscriber as global variable
 
 if flag_Steer == 0:
  Steer_Cont = data.data
  flag_Steer = 1

sub1 = rospy.Subscriber('/Control_Action_Steering', Float64, callback_Control_Steering)
#######################################################################

#######################################################################
#ROS Subscriber Code for Velocity
def callback_Control_Velocity(data):
 global Vel_Cont	#Identify msg variable created as global variable
 global sub2,flag_Vel		#Identify a subscriber as global variable

 if flag_Vel == 0:
  Vel_Cont = data.data
  flag_Vel = 1

sub2 = rospy.Subscriber('/Control_Action_Driving_Velocity', Float64, callback_Control_Velocity)
#######################################################################
#########################################################################################################

#########################################################################################################
#Simulation While Loop
st_time = time.time()
#######################################################################
while 1 and not rospy.is_shutdown():
 end_time = time.time()
 tau  = end_time-st_time
 st_time = time.time()

 if flag_Vel == 1 and flag_Steer == 1:
  #Set the values of the Twist msg to be published
  sentByte = str(Vel_Cont)+','+str(Steer_Cont) 
  Message.write(sentByte)
  print("Control Velocity : ",Vel_Cont)
  print("Control Steering : ",Steer_Cont)
  
  rate.sleep()		#Sleep with rate
#######################################################################
#########################################################################################################
