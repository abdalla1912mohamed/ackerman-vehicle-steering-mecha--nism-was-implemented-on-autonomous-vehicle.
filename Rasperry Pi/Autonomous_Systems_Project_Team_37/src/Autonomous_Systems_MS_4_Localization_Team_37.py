#!/usr/bin/env python

import rospy
import matplotlib.pyplot as plt
from geometry_msgs.msg import Pose
from std_msgs.msg import Float64
import time

import serial

timearray = []
thetaarray = []
i=0
data = Pose()

if __name__ == '__main__':
    rospy.init_node('Autonomous_Systems_MS_4_Localization_Team_37')
    pub1 = rospy.Publisher('/Actual_Reading', Pose, queue_size=10)
    pub2 = rospy.Publisher('/theta', Float64, queue_size=10)

    rate = rospy.Rate(10) # rate of publishing msg 10hz

    Message = serial.Serial('/dev/ttyACM0', 115200)                   # PORT, BAUD RATE
    msg = Message.readline()
    states = str(msg)
    time.sleep(3)

    while 1 and not rospy.is_shutdown():
        msg = Message.readline()
        states = str(msg)

        statesArray = states.split(",")
        try:
            i+=1
            timearray.append(i)
            v = float(statesArray[0])
            x = float(statesArray[1])
            y = float(statesArray[2])
            w = float(statesArray[3])
            thetaarray.append(w)
            data.position.x = x
            data.position.y = y
            data.position.z = v
            data.orientation.z = w
            
        except:
            pass

        pub1.publish(data)	#Publish msg
        pub2.publish(w)

        print(data)
    plt.figure()
    plt.plot(timearray,thetaarray)
    plt.show()

