#!/usr/bin/env python3

# implemented velocity control using joystick input
# more work need here, e.g., position control, force feedback cotrol using FBG sensor input...

import sys
import string

import time
import numpy as np
from actuator_control import actuator_control
from robot_model import robot_model

# from joystick.msg import joystickmsg

from std_msgs import Float64MultiArray


class robot_control():
  """docstring for robot_control"""
  def __init__(self):
    #super(robot_control, self).__init__()
    #self.arg = arg

    rm = robot_model()
    ac = actuator_control() # initialize galil gard

    self.scale = 1/100
    self.sub = rospy.Subscriber("joystick", Float64MultiArray, self.callback)



  def callback(self, data):
    # acutate the robot when having input from the joystick
    delt_x = data[0]
    delt_y = data[1]
    delt_z = data[2]
    button = data[3]
    
    if button == 0.0: # if the button is not pressed, acutate RCM stucture
      w_x = self.scale*delt_x
      w_y = self.scale*delt_y
      v_z = self.scale*delt_z
      vel = np.zeros([1,3])
      vel[0,0] = v_z
      vel[0,1] = w_x
      vel[0,2] = w_y

      joint_position = ac.tell_jointposition()
      J3tip_inv = rm.invjacobian_3tip(joint_position)

      q_dot = J3tip_inv*vel #q4_dot, q5_dot, q6_dot

      #here fill the joint velocities in 1x6
      # joint_vel = [0,0,0, q_dot[0],q_dot[1],q_dot[2]]
      
      joint_vel = [0,0,0, 5, 0,0] #for test

      # command the motors 456 to move in jog mode
      ac.actuator_jog(self, joint_vel)

    elif button == 1.0: # if the button is pressed, actuate SCARA arm
      v_x = self.scale*delt_x
      v_y = self.scale*delt_y
      v_z = self.scale*delt_z
      vel = np.zeros([1,3])
      vel[0,0] = v_z
      vel[0,1] = v_x
      vel[0,2] = v_y

      joint_position = ac.tell_jointposition()
      J3tip_inv = rm.invjacobian_03(joint_position)

      q_dot = J03_inv*vel #q4_dot, q5_dot, q6_dot

      #here fill the joint velocities in 1x6
      # joint_vel = [q_dot[0],q_dot[1],q_dot[2],0,0,0]

      joint_vel = [0,0,0, -5, 0,0] #for test

      # command the motors 456 to move in jog mode
      ac.actuator_jog(self, joint_vel)

    else:
      print(colored('[WARN] incorrect joystick input in robot_control.py', 'yellow'))



def main(args):
  rc = robot_control()
  # rospy.init_node('image_converter', anonymous=True)
  rospy.init_node('robotcontrol_subscriber', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")


if __name__ == '__main__':
    main(sys.argv)
