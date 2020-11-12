#!/usr/bin/env python3

#demo GUI to show key data and plot forces

import rospy
from std_msgs.msg import String
import time
import numpy as np
import csv
from sensor_msgs.msg import Joy
from BeihangEyerobot.msg import floatlist

class logger(object):
	"""docstring for logger"""
	# def __init__(self, arg):
		# super(logger, self).__init__()
		# self.arg = arg
	def __init__(self):
		
		self.jointpos = [0.0,0.0,0.0,0.0,0.0,0.0]
		self.jointvel = [0.0,0.0,0.0,0.0,0.0,0.0]
		self.tippos = [0.0,0.0,0.0,0.0,0.0,0.0] #rotation is represeted as euler in ('zyx', degrees=True)
		self.tipvel = [0.0,0.0,0.0,0.0,0.0,0.0] 

		self.fbgpeaks = [0.0, 0.0, 0.0] #[channal1, channal2, channal3]
		self.fbgforces = [0.0, 0.0, 0.0] #[Fx, Fy, F_norm]

		self.joystick = [0.0, 0.0, 0.0, 0] #[rx, ry, rz, button]

		self.pedal = 'null'

		self.sub_tippos = rospy.Subscriber('robotstate/tippos', floatlist, self.callback_tippos)
		self.sub_jointpos = rospy.Subscriber('robotstate/jointpos', floatlist, self.callback_jointpos)
		self.sub_tipvel = rospy.Subscriber('robotstate/tipvel', floatlist, self.callback_tipvel)
		self.sub_jointvel = rospy.Subscriber('robotstate/jointvel', floatlist, self.callback_jointvel)

		self.sub_fbgforces = rospy.Subscriber('fbg_force', floatlist, self.callback_fbgforces)
		self.sub_fbgpeaks = rospy.Subscriber('fbg_peaks', floatlist, self.callback_fbgpeaks)

		self.sub_pedal = rospy.Subscriber("pedal_value", String, self.callback_pedal)
		
		self.sub_joystick = rospy.Subscriber("joy", Joy, self.callback_joystick)


	def callback_tippos(self, data):
		self.tippos = data.data

	def callback_tipvel(self, data):
		self.tipvel = data.data

	def callback_jointpos(self, data):
		self.jointpos = data.data

	def callback_jointvel(self, data):	
		self.jointvel = data.data


	def callback_fbgforces(self, data):
		self.fbgforces = data.data

	def callback_fbgpeaks(self, data):
		self.fbgpeaks = data.data


	def callback_pedal(self, data):
		self.pedal = data.data


	def callback_joystick(self, data):
		# acutate the robot when having input from the joystick
		self.joystick[0] = data.axes[0]
		self.joystick[1] = data.axes[1]
		self.joystick[2] = data.axes[2]
		self.joystick[3] = data.buttons[0]


	def savedata_csv(self):
		date = time.strftime("%Y-%m-%d-%H_%M_%S",time.localtime(time.time())) 
		filename="/home/b413/robotcontrol_ws/src/BeihangEyerobot/logger/"+date+r" robot logger.csv"
		header = ["timestamp(s)","tippos_x(mm)","tippos_y(mm)","tippos_z(mm)","tippos_rx(rad)","tippos_ry(rad)","tippos_rz(rad)",
				"jointpos1(rad)","jointpos2(rad)","jointpos3(mm)","jointpos4(rad)","jointpos5(rad)","jointpos6(mm)",
				"tipvel_vx","tipvel_vy","tipvel_vz","tipvel_wx","tipvel_wy","tipvel_wz",
				"jointvel1","jointvel2","jointvel3","jointvel4","jointvel5","jointvel6",
				"fbg_Fx(mN)","fbg_Fy(mN)","fbg_Fnorm(mN)","fbg_peak1","fbg_peak2","fbg_peak3",
				"joystick_x","joystick_y","joystick_z","joystick_button","pedal"]
		
		with open(filename,"w") as datacsv:
			csvwriter = csv.writer(datacsv,delimiter=',')
			csvwriter.writerow(header)
			while not rospy.is_shutdown():
				timestamp = [time.time()]
				pedal_value = [self.pedal]
				data = timestamp+self.tippos+self.jointpos+self.tipvel+self.jointvel+self.fbgforces+self.fbgpeaks+self.joystick+pedal_value
				csvwriter.writerow(data)
	

if __name__ == '__main__':

	rospy.init_node('logger', anonymous=True)

	lg = logger()

	lg.savedata_csv()