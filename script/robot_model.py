#!/usr/bin/env python3


import sys
import string
from math import pi
from math import cos
from math import sin
import numpy as np
from actuator_control import actuator_control

class robot_model():
	"""docstring for robot_model"""
	def __init__(self):
	  #super(robot_model, self).__init__()
		#define robot mechanical parameters here
		self.r1 = 200    #unit: mm; distance from scara joint 1 to scara joint 2
		self.r234 = 499  #unit: mm; distance from scara joint 2 to rcm point when the robot arms are alined as a line
		self.rcm_b = 70  #unit: mm; distance from rcm slider joint to middle plane joint
		#self.l_init = 50 #unit: mm; distance from rcm point to rcm structure last pivot
	
	def fwdkin_0tip(self, joint_value, tip_offset):
		# forward kinematics from joint 0 to end effector tip
		# tip offset means xyz offset from end effector tip to rcm point in the rcm coordinate frame
		lx = tip_offset[0]
		ly = tip_offset[1]
		lz = tip_offset[2]
		
		# joint_value = actuator_control.tell_jointposition()
		q1 = joint_value[0]
		q2 = joint_value[1]
		q3 = joint_value[2]
		q4 = joint_value[3]
		q5 = joint_value[4]
		q6 = joint_value[5]


		tx_0tip = self.r234*sin(q1+q2)+self.r1*sin(q1)-(ly*cos(q1+q2)*cos(q4+q5))/2-(lz*cos(q1+q2)*sin(q4+q5))/2+(q6*cos(q1+q2)*sin(q4+q5))/2+lx*cos(q1+q2)*cos(q4)-ly*sin(q1+q2)*cos(q5)-lz*sin(q1+q2)*sin(q5)+q6*sin(q1+q2)*sin(q5)+(ly*cos(q4-q5)*cos(q1+q2))/2-(lz*sin(q4-q5)*cos(q1+q2))/2+(q6*sin(q4-q5)*cos(q1+q2))/2
		ty_0tip =  (q6*sin(q1+q2)*sin(q4+q5))/2-self.r1*cos(q1)-(ly*cos(q4+q5)*sin(q1+q2))/2-(lz*sin(q1+q2)*sin(q4+q5))/2-self.r234*cos(q1+q2)+ly*cos(q1+q2)*cos(q5)+lx*sin(q1+q2)*cos(q4)+lz*cos(q1+q2)*sin(q5)-q6*cos(q1+q2)*sin(q5)+(ly*cos(q4-q5)*sin(q1+q2))/2-(lz*sin(q4-q5)*sin(q1+q2))/2+(q6*sin(q4-q5)*sin(q1+q2))/2
		tz_0tip = q3-lx*sin(q4)-lz*cos(q4)*cos(q5)+q6*cos(q4)*cos(q5)+ly*cos(q4)*sin(q5)
		
		E0tip = np.zeros([4,4])
		E0tip[0,0] = cos(q1+q2)*cos(q4)
		E0tip[0,1] = cos(q1+q2)*sin(q4)*sin(q5)-sin(q1+q2)*cos(q5)
		E0tip[0,2] = sin(q1+q2)*sin(q5)+cos(q1+q2)*cos(q5)*sin(q4)
		E0tip[0,3] = tx_0tip
		
		E0tip[1,0] = sin(q1+q2)*cos(q4)
		E0tip[1,1] = cos(q1+q2)*cos(q5)+sin(q1+q2)*sin(q4)*sin(q5)
		E0tip[1,2] = sin(q1+q2)*cos(q5)*sin(q4)-cos(q1+q2)*sin(q5)
		E0tip[1,3] = ty_0tip
		
		E0tip[2,0] = -sin(q4)
		E0tip[2,1] = cos(q4)*sin(q5)
		E0tip[2,2] = cos(q4)*cos(q5)
		E0tip[2,3] = tz_0tip
		
		E0tip[3,0] = 0
		E0tip[3,1] = 0
		E0tip[3,2] = 0
		E0tip[3,3] = 1

		return E0tip



	def fwdkin_3tip(self, joint_value, tip_offset):
		# forward kinematics from joint 0 to end effector tip
		# tip offset means xyz offset from end effector tip to rcm point in the rcm coordinate frame 
		lx = tip_offset[0]
		ly = tip_offset[1]
		lz = tip_offset[2]

		# joint_value = actuator_control.tell_jointposition()
		q1 = joint_value[0]
		q2 = joint_value[1]
		q3 = joint_value[2]
		q4 = joint_value[3]
		q5 = joint_value[4]
		q6 = joint_value[5]

		tx_3tip = lx*cos(q4)-lz*cos(q5)*sin(q4)+q6*cos(q5)*sin(q4)+ly*sin(q4)*sin(q5)
		ty_3tip = ly*cos(q5)+lz*sin(q5)-q6*sin(q5)
		tz_3tip = q6*cos(q4)*cos(q5)-lz*cos(q4)*cos(q5)-lx*sin(q4)+ly*cos(q4)*sin(q5)
		
		E3tip = np.zeros([4,4])
		E3tip[0,0] = cos(q4)
		E3tip[0,1] = sin(q4)*sin(q5)
		E3tip[0,2] = cos(q5)*sin(q4)
		E3tip[0,3] = tx_3tip
		
		E3tip[1,0] = 0
		E3tip[1,1] = cos(q5)
		E3tip[1,2] = -sin(q5)
		E3tip[1,3] = ty_3tip
		
		E3tip[2,0] = -sin(q4)
		E3tip[2,1] = cos(q4)*sin(q5)
		E3tip[2,2] = cos(q4)*cos(q5)
		E3tip[2,3] = tz_3tip
		
		E3tip[3,0] = 0
		E3tip[3,1] = 0
		E3tip[3,2] = 0
		E3tip[3,3] = 1

		return E3tip

	def fwdkin_03(self, joint_value, tip_offset):
		# forward kinematics from joint 0 to end effector tip
		# tip offset means xyz offset from end effector tip to rcm point in the rcm coordinate frame 
		lx = tip_offset[0]
		ly = tip_offset[1]
		lz = tip_offset[2]

		# joint_value = actuator_control.tell_jointposition()
		q1 = joint_value[0]
		q2 = joint_value[1]
		q3 = joint_value[2]
		q4 = joint_value[3]
		q5 = joint_value[4]
		q6 = joint_value[5]

		tx_03 = self.r234*sin(q1+q2)+self.r1*sin(q1)
		ty_03 = -self.r234*cos(q1+q2)-self.r1*cos(q1)
		tz_03 = q3;
		
		E03 = np.zeros([4,4])
		E03[0,0] = cos(q1+q2)
		E03[0,1] = -sin(q1+q2)
		E03[0,2] = 0
		E03[0,3] = tx_03
		
		E03[1,0] = sin(q1+q2)
		E03[1,1] = cos(q1+q2)
		E03[1,2] = 0
		E03[1,3] = ty_03
		
		E03[2,0] = 0
		E03[2,1] = 0
		E03[2,2] = 1
		E03[2,3] = tz_03
		
		E03[3,0] = 0
		E03[3,1] = 0
		E03[3,2] = 0
		E03[3,3] = 1

		return E03
      
 
	def invkin0tip(self, tip_offset):
		# To do: calculate inverse kinematics for joint 0-tip here
		pass

	def invkin03(self, tip_offset):
		# To do: calculate inverse kinematics for joint 0-3 here
		pass

	def invkin3tip(self, tip_offset):
		# To do: calculate inverse kinematics for joint 3-tip here
		pass

	def jacobian():
		pass

	def invjacobian():
		pass

if __name__ == '__main__':
	ac = actuator_control()
	joint_value = ac.tell_jointposition()
	robot = robot_model()
	tip_offset = [0,0,0]
	E_0tip = robot.fwdkin_0tip(joint_value,tip_offset)
	E_3tip = robot.fwdkin_3tip(joint_value,tip_offset)
	E_03 = robot.fwdkin_03(joint_value,tip_offset)
	print('E_0tip',E_0tip)
	print('E_3tip',E_3tip)
	print('E_03',E_03)
