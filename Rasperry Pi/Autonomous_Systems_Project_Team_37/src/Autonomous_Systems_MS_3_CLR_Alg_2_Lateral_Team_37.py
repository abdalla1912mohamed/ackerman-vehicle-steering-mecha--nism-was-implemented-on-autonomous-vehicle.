#!/usr/bin/env python

#########################################################################################################
#Import the required libraries:
from __future__ import print_function,division
import rospy
from geometry_msgs.msg import Pose,Twist
from std_msgs.msg import Float64
import numpy as np
import math
import time

time.sleep(10)
#########################################################################################################

#########################################################################################################
#######################################################################
#Initialize ROS Node
rospy.init_node('Autonomous_Systems_MS_3_CLR_Alg_2_Lateral_Team_37') #Identify ROS Node
#######################################################################

#######################################################################
#ROS Publisher Code for Steering
pub1 = rospy.Publisher('/Control_Action_Steering', Float64, queue_size=10)
rate = rospy.Rate(10) # rate of publishing msg 10hz
#######################################################################

#######################################################################
#ROS Subscriber Variables
Actual_position = Pose()
Actual_Speed = Twist()
Lane_Des = 0
flag_St = 0
flag_Lat = 0
#######################################################################

#######################################################################
#ROS Subscriber Code for Position and Velocity
def callback_Act_St(data):
 global Actual_position, Actual_Speed	#Identify msg variable created as global variable
 global sub1,flag_St		#Identify a subscriber as global variable
 
 if flag_St == 0:
  Actual_position.position.x = data.position.x
  Actual_position.position.y = data.position.y
  Actual_position.orientation.z = data.orientation.z
  Actual_Speed.linear.x = data.position.z
  flag_St = 1

sub1 = rospy.Subscriber('/Actual_Reading', Pose, callback_Act_St)
#######################################################################

#######################################################################
#ROS Subscriber Code for Position and Velocity
def callback_Des_Lane(data):
 global Lane_Des	#Identify msg variable created as global variable
 global sub2,flag_Lat	#Identify a subscriber as global variable
 
 if flag_Lat == 0:
  Lane_Des = data.data
  print('Lane_Des = '+str(Lane_Des))
  flag_Lat = 1
  
sub2 = rospy.Subscriber('/Lateral_Distance', Float64, callback_Des_Lane)
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
def Pure_Pursuit_Control(Lane_Des,Pos_Act,V_Act):
  K_p = 0.5
  global Wheel_Base
  x_Lane = Pos_Act[0] + 1*np.sign(V_Act)#K_p*V_Act
  
  Lookahead_pnt = [x_Lane,Lane_Des]
  print('Lookahead_pnt = '+str(Lookahead_pnt))
  dx = Lookahead_pnt[0]-Pos_Act[0]
  dy = Lookahead_pnt[1]-Pos_Act[1]
  L_d = np.sqrt((dx)**2 + (dy)**2)
  alpha = - Pos_Act[2] + np.arctan2(dy,dx)
  
  Steer_Cont = np.arctan(((2*Wheel_Base*np.sin(alpha))/(L_d)))
  
  if(abs(Steer_Cont) >= np.radians(30)):
   Steer_Cont = np.sign(Steer_Cont)*np.radians(30)
  print('Steer_Cont = '+str(Steer_Cont))
  return Steer_Cont
#########################################################################################################

#########################################################################################################
def Stanley_Control(Lane_Des,Pos_Act,V_Act):
  K_p = 0.5
  K_v = 1
  global Wheel_Base
  
  x_Lane = Pos_Act[0] + 1*np.sign(V_Act)#K_p*V_Act
  Lookahead_pnt = [x_Lane,Lane_Des]
  print('Lookahead_pnt = '+str(Lookahead_pnt))
  dx = Lookahead_pnt[0]-(Pos_Act[0]+Wheel_Base*np.cos(Pos_Act[2]))
  dy = Lookahead_pnt[1]-(Pos_Act[1]+Wheel_Base*np.sin(Pos_Act[2]))
  L_d = np.sqrt((dx)**2 + (dy)**2)
  
  th_p = 0 - Pos_Act[2] 
  try:
   Steer_Cont = th_p + np.arctan(((K_v*dy)/(V_Act)))
  except:
   Steer_Cont = 0
  if(abs(Steer_Cont) >= np.radians(30)):
   Steer_Cont = np.sign(Steer_Cont)*np.radians(30)
  print('Steer_Cont = '+str(Steer_Cont))
  return Steer_Cont
#########################################################################################################

#########################################################################################################
Lateral_Controller = rospy.get_param("~Lateral_Controller")
Wheel_Base = rospy.get_param("~Wheel_Base") #Check Gazebo Model Dimenssions 
#######################################################################
#Simulation While Loop
st_time = time.time()
Steer_Cont = 0
#######################################################################
while 1 and not rospy.is_shutdown():
 st_time = time.time()
 
 if flag_St == 1 and flag_Lat == 1:
  V_Act = Actual_Speed.linear.x
  print('Vel_Act = '+str(V_Act))
 
  Angles_Act = quaternion_to_euler(Actual_position.orientation.x, Actual_position.orientation.y, Actual_position.orientation.z, Actual_position.orientation.w)
  Yaw_act = Angles_Act[0] 
  Pos_Act = [Actual_position.position.x, Actual_position.position.y,Yaw_act]
  print('Pos_Act = '+str(Pos_Act))
 
  if Lateral_Controller in "Pure_Pursuit":
   Steer_Cont = Pure_Pursuit_Control(Lane_Des,Pos_Act,V_Act)
  elif Lateral_Controller in "Stanley":
   Steer_Cont = Stanley_Control(Lane_Des,Pos_Act,V_Act)
 
  flag_St = 0 
  flag_Lat = 0
   
 pub1.publish(Steer_Cont)	#Publish msg
 rate.sleep()		#Sleep with rate
 end_time = time.time()
 tau  = end_time-st_time
 print(tau)
#######################################################################
#########################################################################################################

