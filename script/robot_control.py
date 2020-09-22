#!/usr/bin/env python3


import sys
import string

import time
import actuator_control

class robot_control():
	"""docstring for robot_control"""
	def __init__(self, arg):
		super(robot_control, self).__init__()
		self.arg = arg
		

    def joint2actuator(q1,q2,q3,q4,q5,q6): #q1 for scara arm...q6 for rcm small screw motor
        a1 = q1*10000 #unit: degree  ---temp value, need update!!
        a2 = q2*10000 #unit: degree  ---temp value, need update!!
        a3 = q3*10000 #unit: mm
        a4 = q4*2500  #unit: degree
        a5 = q5*4000  #unit: mm
        a6 = q6*2048  #unit: mm  for RE16 motor --need update!!
        return a1, a2, a3, a4, a5, a6


#runs main() if example.py called from the console
if __name__ == '__main__':
  rc = robot_control
