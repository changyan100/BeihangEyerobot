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
      
 
	def invkin_0tip(self, tip_offset):
		# To do: calculate inverse kinematics for joint 0-tip here
		pass

	def invkin_03(self, tip_offset):
		# To do: calculate inverse kinematics for joint 0-3 here
		pass

	def invkin_3tip(self, tip_offset):
		# To do: calculate inverse kinematics for joint 3-tip here
		pass

	# Use with caution!!! Have Not been evalauted!!!
	def jacobian_0tip(self, joint_value):
		# Jacobian matrix from joint 0 to end effector tip
		q1 = joint_value[0]
		q2 = joint_value[1]
		q3 = joint_value[2]
		q4 = joint_value[3]
		q5 = joint_value[4]
		q6 = joint_value[5]

		j11 = self.r234*cos(q1+q2)+self.r1*cos(q1)-(q6*sin(q1+q2)*sin(q4+q5))/2+q6*cos(q1+q2)*sin(q5)-(q6*sin(q4-q5)*sin(q1+q2))/2
		j12 = self.r234*cos(q1+q2)-(q6*sin(q1+q2)*sin(q4+q5))/2+q6*cos(q1+q2)*sin(q5)-(q6*sin(q4-q5)*sin(q1+q2))/2
		j15 = sin(q4)*((q6*sin(q1+q2)*sin(q4+q5))/2-q6*cos(q1+q2)*sin(q5)+(q6*sin(q4-q5)*sin(q1+q2))/2)+q6*sin(q1+q2)*cos(q4)**2*cos(q5)
		j21 = self.r234*sin(q1+q2)+self.r1*sin(q1)+(q6*cos(q1+q2)*sin(q4+q5))/2+q6*sin(q1+q2)*sin(q5)+(q6*sin(q4-q5)*cos(q1+q2))/2
		j22 = self.r234*sin(q1+q2)+(q6*cos(q1+q2)*sin(q4+q5))/2+q6*sin(q1+q2)*sin(q5)+(q6*sin(q4-q5)*cos(q1+q2))/2
		j25 = -sin(q4)*((q6*cos(q1+q2)*sin(q4+q5))/2+q6*sin(q1+q2)*sin(q5)+(q6*sin(q4-q5)*cos(q1+q2))/2)-q6*cos(q1+q2)*cos(q4)**2*cos(q5)
		j34 = -cos(q1+q2)*((q6*cos(q1+q2)*sin(q4+q5))/2+q6*sin(q1+q2)*sin(q5)+(q6*sin(q4-q5)*cos(q1+q2))/2)-sin(q1+q2)*((q6*sin(q1+q2)*sin(q4+q5))/2-q6*cos(q1+q2)*sin(q5)+(q6*sin(q4-q5)*sin(q1+q2))/2)
		j35 = cos(q1+q2)*cos(q4)*((q6*sin(q1+q2)*sin(q4+q5))/2-q6*cos(q1+q2)*sin(q5)+(q6*sin(q4-q5)*sin(q1+q2))/2)-sin(q1+q2)*cos(q4)*((q6*cos(q1+q2)*sin(q4+q5))/2+q6*sin(q1+q2)*sin(q5)+(q6*sin(q4-q5)*cos(q1+q2))/2)

		J06 = np.zeros([6,6])
		J06[0,0] = j11
		J06[0,1] = j12
		J06[0,2] = 0
		J06[0,3] = q6*cos(q1+q2)*cos(q4)*cos(q5)
		J06[0,4] = j15
		J06[0,5] = sin(q1+q2)*sin(q5)+cos(q1+q2)*cos(q5)*sin(q4)

		J06[1,0] = j21
		J06[1,1] = j22
		J06[1,2] = 0
		J06[1,3] = q6*sin(q1+q2)*cos(q4)*cos(q5)
		J06[1,4] = j25
		J06[1,5] = sin(q1+q2)*cos(q5)*sin(q4)-cos(q1+q2)*sin(q5)

		J06[2,0] = 0
		J06[2,1] = 0
		J06[2,2] = 1
		J06[2,3] = j34
		J06[2,4] = j35
		J06[2,5] = cos(q4)*cos(q5)

		J06[3,0] = 0
		J06[3,1] = 0
		J06[3,2] = 0
		J06[3,3] = -sin(q1+q2)
		J06[3,4] = cos(q1+q2)*cos(q4)
		J06[3,5] = 0

		J06[4,0] = 0
		J06[4,1] = 0
		J06[4,2] = 0
		J06[4,3] = cos(q1+q2)
		J06[4,4] = sin(q1+q2)*cos(q4)
		J06[4,5] = 0

		J06[5,0] = 1
		J06[5,1] = 1
		J06[5,2] = 0
		J06[5,3] = 0
		J06[5,4] = -sin(q4)
		J06[5,5] = 0
		
		return J06

	# Use with caution!!! Have Not been evalauted!!!
	def invjacobian_0tip(self, joint_value):
		J06 = self.jacobian_0tip(joint_value)
		J06_inv = np.linalg.inv(J06)
		return J06_inv


	def jacobian_03(self, joint_value):
		# Jacobian matrix from joint 0 to 3
		# [vx, vy, vz] = J03*[q1_dot, q2_dot, q3_dot]
		q1 = joint_value[0]
		q2 = joint_value[1]
		q3 = joint_value[2]
		q4 = joint_value[3]
		q5 = joint_value[4]
		q6 = joint_value[5]

		J03 = np.zeros([3,3])
		J03[0,0] = self.r1*cos(q1)+self.r234*cos(q1+q2)
		J03[0,1] = self.r234*cos(q1+q2)
		J03[0,2] = 0

		J03[1,0] = self.r1*sin(q1)+self.r234*sin(q1+q2) 
		J03[1,1] = self.r234*sin(q1+q2)
		J03[1,2] = 0

		J03[2,0] = 0
		J03[2,1] = 0
		J03[2,2] = 1

		return J03

	def invjacobian_03(self, joint_value):
		# Inverse Jacobian matrix from joint 0 to 3
		# J03_inv*[vx, vy, vz] = [q1_dot, q2_dot, q3_dot]
		J03 = self.jacobian_03(joint_value)
		# print("J03 = ", J03)
		J03_inv = np.linalg.inv(J03)
		return J03_inv


	def invjacobian_03_direct_value(self, tip_vel, joint_value):
		# direct calculate joint velcoties for 0 1 2 joints
		x_dot = tip_vel[0,0]
		y_dot = tip_vel[1,0]
		z_dot = tip_vel[2,0]
		q1 = joint_value[0]
		q2 = joint_value[1]
		q3 = joint_value[2]

		M1 = self.r1*cos(q1)+self.r1*sin(q1)+self.r234*cos(q1+q2)+self.r234*sin(q1+q2)
		N1 = self.r234*cos(q1+q2)+self.r234*sin(q1+q2)
		M2 = self.r1*cos(q1)-self.r1*sin(q1)+self.r234*cos(q1+q2)-self.r234*sin(q1+q2)
		N2 = self.r234*cos(q1+q2)-self.r234*sin(q1+q2)
		q2_dot = (x_dot-y_dot-M2/M1*(x_dot+y_dot))/(-M2*N1/M1+N2)
		q1_dot = (x_dot+y_dot)/M1-N1/M1*((x_dot-y_dot-M2/M1*(x_dot+y_dot))/(-M2*N1/M1+N2))
		q3_dot = z_dot
		q_dot = [q1_dot, q2_dot, q3_dot, 0,0,0]
		return q_dot

	def jacobian_3tip(self, joint_value):
		# Jacobian matrix from joint 3 to end effector tip
		# [vz, wx, wy] = J3tip*[q4_dot, q5_dot, q6_dot]
		q1 = joint_value[0]
		q2 = joint_value[1]
		q3 = joint_value[2]
		q4 = joint_value[3]
		q5 = joint_value[4]
		q6 = joint_value[5]

		J3tip = np.zeros([3,3])
		J3tip[0,0] = 0
		J3tip[0,1] = 0
		J3tip[0,2] = 1

		J3tip[1,0] = 0 
		J3tip[1,1] = 1
		J3tip[1,2] = 0

		J3tip[2,0] = 1
		J3tip[2,1] = 0
		J3tip[2,2] = 0

		return J3tip

	def invjacobian_3tip(self, joint_value):
		# Inverse Jacobian matrix from joint 3 to end effector tip
		# J3tip_inv*[vz, wx, wy] = [q4_dot, q5_dot, q6_dot]		
		J3tip = self.jacobian_3tip(joint_value)
		J3tip_inv = np.linalg.inv(J3tip)
		return J3tip_inv



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
