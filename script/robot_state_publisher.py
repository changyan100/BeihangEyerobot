#!/usr/bin/env python3

# publish robot position and velocity to topic /robotstate/xx.

import sys
import string

import time
import numpy as np
from actuator_control import actuator_control
from robot_model import robot_model
import rospy

from sensor_msgs.msg import Joy
from std_msgs.msg import String
from BeihangEyerobot.msg import floatlist

from math import pi
from termcolor import colored  #for print in color

from scipy.spatial.transform import Rotation as R


def talker():
    rm = robot_model()
    ac = actuator_control('OFF')


    pub_tippos = rospy.Publisher('robotstate/tippos', floatlist, queue_size=10)
    pub_jointpos = rospy.Publisher('robotstate/jointpos', floatlist, queue_size=10)
    pub_tipvel = rospy.Publisher('robotstate/tipvel', floatlist, queue_size=10)
    pub_jointvel = rospy.Publisher('robotstate/jointvel', floatlist, queue_size=10)
    rospy.init_node('robotstate_publisher', anonymous=True)
    rate = rospy.Rate(1000) # 1000hz
    while not rospy.is_shutdown():
        
        #calculate robot joint postions
        jointpos = ac.tell_jointposition()
        #calculate robot tip positions //check tip offset parameters!!!
        tip_offset = [0,0,0]
        E_0tip = rm.fwdkin_0tip(jointpos, tip_offset)
        tippos = [E_0tip[0,3], E_0tip[1,3], E_0tip[2,3]]
        #covert rotation matrix to euler 
        r = R.from_matrix(E_0tip[0:3,0:3])
        euler = r.as_euler('zyx', degrees=True)
        tippos.append(euler[0])
        tippos.append(euler[1])
        tippos.append(euler[2])
        #calculate robot joint velocities
        jointvel = ac.tell_jointvelocity()
        #calculate robot tip velocities
        J0tip = rm.jacobian_0tip(jointpos)
        tipvel = np.dot(J0tip, jointvel)

        # rospy.loginfo(hello_str)
        
        pub_tippos.publish(tippos)
        pub_jointpos.publish(jointpos)
        pub_tipvel.publish(tipvel)
        pub_jointvel.publish(jointvel)
        
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass