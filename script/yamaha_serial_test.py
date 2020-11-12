#!/usr/bin/env python3


import time
import serial

# configure the serial connections (the parameters differs on the device you are connecting to)
ser = serial.Serial(
	port='/dev/ttyUSB0',
	baudrate=9600,
	parity=serial.PARITY_ODD,
	stopbits=serial.STOPBITS_ONE,
	bytesize=serial.EIGHTBITS
)

# ser.open()
ser.isOpen()

print('Enter your commands below.\r\nInsert "exit" to leave the application.')

input_keyboard=1
while 1 :
	# get keyboard input
	input_keyboard = input(">> ")
        # Python 3 users
        # input = input(">> ")
	if input_keyboard == 'exit':
		ser.close()
		exit()
	else:
		# send the character to the device
		# (note that I happend a \r\n carriage return and line feed to the characters - this is requested by my device)
		ser.write(input_keyboard + '\r\n')
		out = ''
		# let's wait one second before reading output (let's give device time to answer)
		time.sleep(1)
		while ser.inWaiting() > 0:
			out += ser.read(1)
			
		if out != '':
			print(">>" + out)