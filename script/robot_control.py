#!/usr/bin/env python3

# implemented velocity control using joystick input
# more work need here, e.g., position control, force feedback cotrol using FBG sensor input...

import sys
import string

import time
import numpy as np
from actuator_control import actuator_control
from robot_model import robot_model
import rospy

from sensor_msgs.msg import Joy
from std_msgs.msg import String

from math import pi
from termcolor import colored  #for print in color


# from joystick.msg import joystickmsg

# from std_msgs.msg import Float32MultiArray


class robot_control():
  """docstring for robot_control"""
  def __init__(self):
    #super(robot_control, self).__init__()
    #self.arg = arg

    self.rm = robot_model()
    self.ac = actuator_control('ON') # initialize galil gard and servo on

    self.scale_rcm = [1, 0.06, 0.5]
    self.scale_scara = [10,10,4]

    self.pedal_value = "null"

    self.sub_pedal = rospy.Subscriber("pedal_value", String, self.callback_pedal)
    self.sub_joystick = rospy.Subscriber("joy", Joy, self.callback_joystick)

    self.force_error1 = 0
    self.m_PID_accumate = 0

  def callback_pedal(self, data):
    self.pedal_value = data.data
    print("pedal = ", self.pedal_value)


  def callback_joystick(self, data):
    # acutate the robot when having input from the joystick
    delt_x = data.axes[0]
    delt_y = data.axes[1]
    delt_z = data.axes[2]
    button = int(data.buttons[0])

    print('button = ',button, 'delt_x,y,z = ', delt_x, delt_y, delt_z)
    
    
    # if self.pedal_value == "left": # if the button is not pressed, acutate RCM stucture
    if button == 0:
      w_x = delt_x
      w_y = self.scale_rcm[1]*delt_y
      v_z = self.scale_rcm[2]*delt_z
      vel = np.zeros([3,1])
      vel[0,0] = v_z
      vel[1,0] = w_x
      vel[2,0] = w_y

      joint_position = self.ac.tell_jointposition()
      J3tip_inv = self.rm.invjacobian_3tip(joint_position)

      q_dot = np.dot(J3tip_inv, vel) #q4_dot, q5_dot, q6_dot
      # print('J3tip_inv', J3tip_inv)
      # print('q_dot', q_dot)

      #here fill the joint velocities in 1x6
      joint_vel = [0,0,0, q_dot[0,0]*self.scale_rcm[0], q_dot[1,0]*self.scale_rcm[1], q_dot[2,0]*self.scale_rcm[2]]

      
      # joint_vel = [0,0,0, 0, 0, 0] #for test

      # command the motors 456 to move in jog mode
      self.ac.actuator_jog(joint_vel)

    # elif self.pedal_value == "right": # if the button is pressed, actuate SCARA arm
    elif button == 1: # if the button is pressed, actuate SCARA arm
      v_x = self.scale_scara[0]*delt_y
      v_y = self.scale_scara[1]*delt_x
      v_z = self.scale_scara[2]*delt_z
      vel = np.zeros([3,1])
      vel[0,0] = v_x
      vel[1,0] = v_y
      vel[2,0] = v_z

      joint_position = self.ac.tell_jointposition()
      
      # J03_inv = self.rm.invjacobian_03(joint_position)
      # q_dot = np.dot(J03_inv,vel) #q4_dot, q5_dot, q6_dot
      # #here fill the joint velocities in 1x6
      # joint_vel = [q_dot[0,0],q_dot[1,0],q_dot[2,0],0,0,0]


      #calculate joint_vel directly
      joint_vel = self.rm.invjacobian_03_direct_value(vel, joint_position)

      # joint_vel = [0,0,0, 0, 0, 0] #for test

      # command the motors 456 to move in jog mode
      self.ac.actuator_jog(joint_vel)

    else:
      joint_vel = [0,0,0, 0, 0, 0]
      self.ac.actuator_jog(joint_vel)
      # print(colored('[WARN] incorrect joystick input in robot_control.py', 'yellow'))

  # def impedencecontrol():
  #     atiforce = []
  #     sincetime = time.time()

  #     from datetime import datetime
  #     import time
  #     # 每n秒执行一次
  #     def timer(n):  # 50ms
  #         while True:
  #             print(datetime.now().strftime("%Y-%m-%d  %H:%M:%S"))
  #             freq = xx  
  #             deltime = time.time()-sincetime
  #             magnitude = yy
  #             offset = yy
  #             m_FuZhi = xxx

  #             m_kp = kp
  #             m_ki = ki


  #             force_desired = sin(2*pi*freq*deltime)*magnitude+offset

  #             if (freq == 0)
            
  #               hyCon = m_FuZhi
    
  #             else
    
  #               hyCon = 3.57*m_FuZhi*sin(2*M_PI*freq*deltime)

  #             force_real = atiforce()

  #             force_error = force_desired-force_real

  #             control1 = m_kp * (force_error - m_ierr) + m_ki * self.force_error;
  #             self.m_PID_accumate = control1+  self.m_PID_accumate

  #             self.err2 = self.ierr1
  #             self.ierr1 = force_err

  #             control2 = hyCon + m_PID_add + control1  #unit: mm

  #             joint_pos = [0,0,control2,0,0,0]
  #             actuate_control(joint_pos)

          


  #             time.sleep(n)



  #     timer(5)

def main(args):
  rc = robot_control()
  rospy.init_node('robotcontrol_subscriber', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Program is aborted by the user!")
    rc.ac.actuator_stop()
    rc.ac.actuator_off()
    rc.ac.disconnect()
    rospy.signal_shutdown("Program aborted!")
    



if __name__ == '__main__':
    main(sys.argv)
