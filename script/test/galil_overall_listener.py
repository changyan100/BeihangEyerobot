#!/usr/bin/env python3
# REMEMBER: chmod +x XX.py

import rospy
from galil_command import g_control
# use omni customed message type
from omni_msgs.msg import OmniState
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray


def callback1(data):
    rospy.loginfo(rospy.get_caller_id() + " python: Galil heard Omni")
    # rospy.loginfo(data)

    v1 = data.velocity.x
    v2 = data.velocity.y
    v3 = data.velocity.z

    # TODO: uncomment and test
    # Galil velocity control in joint space
    # v1 -> theta3, v2 -> theta2, v3 -> d6
    # g.g_jog(v1, v2, v3)

    # detect button change
    if data.close_gripper == True and g.grip_state == False:
        grip_count += 1
    # record previous button state
    g.grip_state = data.close_gripper
    # TODO: determine hardware I/O number and revise galil_command.py
    # g.grip(grip_count%2)


def callback2(data):
    rospy.loginfo(rospy.get_caller_id() + " python: Galil heard Omni")
    #rospy.loginfo(data)

    # rotation about its z axis
    roll = data.position[5]
    if g.roll_his != 99999:
        # increment control with 100hz
        incre = (roll - g.roll_his) / 100
        # TODO: cofirm offset and ratio
        # roll = (roll + offset) * ratio_roll
        # TODO: uncomment and test
        # g.iap(distance=roll)
    # record roll history
    g.roll_his = roll
    # publish incre and grip control value
    galil_talker([incre, g.grip_state])


def galil_listener():
    rospy.Subscriber("phantom/state", OmniState, callback1)
    rospy.Subscriber("phantom/joint_state", JointState, callback2)
    rospy.spin()


def galil_talker(data):
    '''
    return speed of roll joint
    '''
    pub_galil = rospy.Publisher('galil/speed', Float32MultiArray, queue_size=10)

    position = Float32MultiArray(data=data)
    pub_galil.publish(position)


if __name__ == '__main__':
    # ROS init
    rospy.init_node('Galil_overall_listener', anonymous=False)

    # retrieve parameter from .launch file
    galil_address = rospy.get_param("~Galil_address")

    # galil card connection
    g = g_control(galil_address)
    grip_count = 0
    # TODO: uncomment for real test
    # g.connect()

    galil_listener()

    # Close galil connection
    # TODO: uncomment for real test
    # g.disconnect()
