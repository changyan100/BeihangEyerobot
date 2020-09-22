#!/usr/bin/env python3


import sys
import string
from math import pi
from math import cos
from math import sin
import numpy as np

class robot_model():
	"""docstring for robot_model"""
	def __init__(self, arg):
	  #super(robot_model, self).__init__()
		#define robot mechanical parameters here
		self.r1 = 200    #unit: mm; distance from scara joint 1 to scara joint 2
		self.r234 = 499  #unit: mm; distance from scara joint 2 to rcm point when the robot arms are alined as a line
		self.rcm_b = 70  #unit: mm; distance from rcm slider joint to middle plane joint

	def fwdkin_0tip(self, tip_offset):
		# forward kinematics from joint 0 to end effector tip
		# tip offset means xyz offset from end effector tip to rcm point in the rcm coordinate frame
		# to facili  
		lx = tip_offset[0]
		ly = tip_offset[1]
		lz = tip_offset[2]
		
		tx_0tip = self.r234*sin(q1 + q2) + self.r1*sin(q1) - (ly*cos(q1 + q2)*cos(q4 + q5))/2 - (lz*cos(q1 + q2)*sin(q4 + q5))/2 + (q6*cos(q1 + q2)*sin(q4 + q5))/2 + lx*cos(q1 + q2)*cos(q4) - ly*sin(q1 + q2)*cos(q5) - lz*sin(q1 + q2)*sin(q5) + q6*sin(q1 + q2)*sin(q5) + (ly*cos(q4 - q5)*cos(q1 + q2))/2 - (lz*sin(q4 - q5)*cos(q1 + q2))/2 + (q6*sin(q4 - q5)*cos(q1 + q2))/2
		ty_0tip =  (q6*sin(q1 + q2)*sin(q4 + q5))/2 - self.r1*cos(q1) - (ly*cos(q4 + q5)*sin(q1 + q2))/2 - (lz*sin(q1 + q2)*sin(q4 + q5))/2 - self.r234*cos(q1 + q2) + ly*cos(q1 + q2)*cos(q5) + lx*sin(q1 + q2)*cos(q4) + lz*cos(q1 + q2)*sin(q5) - q6*cos(q1 + q2)*sin(q5) + (ly*cos(q4 - q5)*sin(q1 + q2))/2 - (lz*sin(q4 - q5)*sin(q1 + q2))/2 + (q6*sin(q4 - q5)*sin(q1 + q2))/2
		tz_0tip = q3 - lx*sin(q4) - lz*cos(q4)*cos(q5) + q6*cos(q4)*cos(q5) + ly*cos(q4)*sin(q5);
		E0tip = np.zeros([4,4])
		E0tip[0,0] = cos(q1 + q2)*cos(q4)
		E0tip[0,1] = cos(q1 + q2)*sin(q4)*sin(q5) - sin(q1 + q2)*cos(q5)
		E0tip[0,2] = sin(q1 + q2)*sin(q5) + cos(q1 + q2)*cos(q5)*sin(q4)
		E0tip[0,3] = tx_0tip
		
		E0tip[1,0] = sin(q1 + q2)*cos(q4)
		E0tip[1,1] = cos(q1 + q2)*cos(q5) + sin(q1 + q2)*sin(q4)*sin(q5)
		E0tip[1,2] = sin(q1 + q2)*cos(q5)*sin(q4) - cos(q1 + q2)*sin(q5)
		E0tip[1,3] = ty_0tip
		
		E0tip[2,0] = -sin(q4)
		E0tip[2,1] = cos(q4)*sin(q5)
		E0tip[2,2] = cos(q4)*cos(q5)
		E0tip[2,3] = tz_0tip
		
		E0tip[3,0] = 0
		E0tip[3,1] = 0
		E0tip[3,2] = 0
		E0tip[3,3] = 1

	def fwdkin_03(self, tip_offset):
		# forward kinematics from joint 0 to end effector tip
		# tip offset means xyz offset from end effector tip to rcm point in the rcm coordinate frame 
		lx = tip_offset[0]
		ly = tip_offset[1]
		lz = tip_offset[2]

		tx_0tip = self.r234*sin(q1 + q2) + self.r1*sin(q1) - (ly*cos(q1 + q2)*cos(q4 + q5))/2 - (lz*cos(q1 + q2)*sin(q4 + q5))/2 + (q6*cos(q1 + q2)*sin(q4 + q5))/2 + lx*cos(q1 + q2)*cos(q4) - ly*sin(q1 + q2)*cos(q5) - lz*sin(q1 + q2)*sin(q5) + q6*sin(q1 + q2)*sin(q5) + (ly*cos(q4 - q5)*cos(q1 + q2))/2 - (lz*sin(q4 - q5)*cos(q1 + q2))/2 + (q6*sin(q4 - q5)*cos(q1 + q2))/2
		ty_0tip =  (q6*sin(q1 + q2)*sin(q4 + q5))/2 - self.r1*cos(q1) - (ly*cos(q4 + q5)*sin(q1 + q2))/2 - (lz*sin(q1 + q2)*sin(q4 + q5))/2 - self.r234*cos(q1 + q2) + ly*cos(q1 + q2)*cos(q5) + lx*sin(q1 + q2)*cos(q4) + lz*cos(q1 + q2)*sin(q5) - q6*cos(q1 + q2)*sin(q5) + (ly*cos(q4 - q5)*sin(q1 + q2))/2 - (lz*sin(q4 - q5)*sin(q1 + q2))/2 + (q6*sin(q4 - q5)*sin(q1 + q2))/2
		tz_0tip = q3 - lx*sin(q4) - lz*cos(q4)*cos(q5) + q6*cos(q4)*cos(q5) + ly*cos(q4)*sin(q5);
		E0tip = np.zeros([4,4])
		E0tip[0,0] = cos(q1 + q2)*cos(q4)
		E0tip[0,1] = cos(q1 + q2)*sin(q4)*sin(q5) - sin(q1 + q2)*cos(q5)
		E0tip[0,2] = sin(q1 + q2)*sin(q5) + cos(q1 + q2)*cos(q5)*sin(q4)
		E0tip[0,3] = tx_0tip
		
		E0tip[1,0] = sin(q1 + q2)*cos(q4)
		E0tip[1,1] = cos(q1 + q2)*cos(q5) + sin(q1 + q2)*sin(q4)*sin(q5)
		E0tip[1,2] = sin(q1 + q2)*cos(q5)*sin(q4) - cos(q1 + q2)*sin(q5)
		E0tip[1,3] = ty_0tip
		
		E0tip[2,0] = -sin(q4)
		E0tip[2,1] = cos(q4)*sin(q5)
		E0tip[2,2] = cos(q4)*cos(q5)
		E0tip[2,3] = tz_0tip
		
		E0tip[3,0] = 0
		E0tip[3,1] = 0
		E0tip[3,2] = 0
		E0tip[3,3] = 1

	def fwdkin_3tip(self, tip_offset):
		# forward kinematics from joint 0 to end effector tip
		# tip offset means xyz offset from end effector tip to rcm point in the rcm coordinate frame 
		lx = tip_offset[0]
		ly = tip_offset[1]
		lz = tip_offset[2]

		tx_0tip = self.r234*sin(q1 + q2) + self.r1*sin(q1) - (ly*cos(q1 + q2)*cos(q4 + q5))/2 - (lz*cos(q1 + q2)*sin(q4 + q5))/2 + (q6*cos(q1 + q2)*sin(q4 + q5))/2 + lx*cos(q1 + q2)*cos(q4) - ly*sin(q1 + q2)*cos(q5) - lz*sin(q1 + q2)*sin(q5) + q6*sin(q1 + q2)*sin(q5) + (ly*cos(q4 - q5)*cos(q1 + q2))/2 - (lz*sin(q4 - q5)*cos(q1 + q2))/2 + (q6*sin(q4 - q5)*cos(q1 + q2))/2
		ty_0tip =  (q6*sin(q1 + q2)*sin(q4 + q5))/2 - self.r1*cos(q1) - (ly*cos(q4 + q5)*sin(q1 + q2))/2 - (lz*sin(q1 + q2)*sin(q4 + q5))/2 - self.r234*cos(q1 + q2) + ly*cos(q1 + q2)*cos(q5) + lx*sin(q1 + q2)*cos(q4) + lz*cos(q1 + q2)*sin(q5) - q6*cos(q1 + q2)*sin(q5) + (ly*cos(q4 - q5)*sin(q1 + q2))/2 - (lz*sin(q4 - q5)*sin(q1 + q2))/2 + (q6*sin(q4 - q5)*sin(q1 + q2))/2
		tz_0tip = q3 - lx*sin(q4) - lz*cos(q4)*cos(q5) + q6*cos(q4)*cos(q5) + ly*cos(q4)*sin(q5);
		E0tip = np.zeros([4,4])
		E0tip[0,0] = cos(q1 + q2)*cos(q4)
		E0tip[0,1] = cos(q1 + q2)*sin(q4)*sin(q5) - sin(q1 + q2)*cos(q5)
		E0tip[0,2] = sin(q1 + q2)*sin(q5) + cos(q1 + q2)*cos(q5)*sin(q4)
		E0tip[0,3] = tx_0tip
		
		E0tip[1,0] = sin(q1 + q2)*cos(q4)
		E0tip[1,1] = cos(q1 + q2)*cos(q5) + sin(q1 + q2)*sin(q4)*sin(q5)
		E0tip[1,2] = sin(q1 + q2)*cos(q5)*sin(q4) - cos(q1 + q2)*sin(q5)
		E0tip[1,3] = ty_0tip
		
		E0tip[2,0] = -sin(q4)
		E0tip[2,1] = cos(q4)*sin(q5)
		E0tip[2,2] = cos(q4)*cos(q5)
		E0tip[2,3] = tz_0tip
		
		E0tip[3,0] = 0
		E0tip[3,1] = 0
		E0tip[3,2] = 0
		E0tip[3,3] = 1

 

	def invkin():

	def jacobian():


	def invjacobian():


if __name__ == '__main__':
  pass
