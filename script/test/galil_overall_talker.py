#!/usr/bin/env python3
# REMEMBER: chmod +x XX.py

import rospy
from galil_command import g_control
from std_msgs.msg import Float32MultiArray


def galil_talker():
    '''
    return joint positions
    '''
    pub_galil = rospy.Publisher('galil/position', Float32MultiArray, queue_size=10)
    rate = rospy.Rate(11) # 11hz
    while not rospy.is_shutdown():
        # TODO: test and retrieve value of joints in degree/mm unit
        # data = g.tp() data must be processed to be array-like
        data = [11, 22, 33, 44]
        position = Float32MultiArray(data=data)
        pub_galil.publish(position)
        rate.sleep()


if __name__ == '__main__':
    # ROS init
    rospy.init_node('Galil_overall_talker', anonymous=False)

    # retrieve parameter from .launch file
    galil_address = rospy.get_param("~Galil_address")

    # galil card connection
    g = g_control(galil_address)
    # TODO: uncomment for real test
    # g.connect()

    galil_talker()

    # Close galil connection
    # TODO: uncomment for real test
    # g.disconnect()
