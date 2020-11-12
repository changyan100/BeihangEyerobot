#!/usr/bin/env python3

import rospy
from std_msgs.msg import String

import serial, time
#initialization and open the port

#possible timeout values:
#    1. None: wait forever, block call
#    2. 0: non-blocking mode, return immediately
#    3. x, x is bigger than 0, float allowed, timeout block call

ser = serial.Serial()
ser.port = "/dev/ttyUSB0"

ser.baudrate = 9600
ser.bytesize = serial.EIGHTBITS #number of bits per bytes
ser.parity = serial.PARITY_ODD #set parity check: no parity
ser.stopbits = serial.STOPBITS_ONE #number of stop bits
#ser.timeout = None          #block read
ser.timeout = 1            #non-block read
#ser.timeout = 2              #timeout block read
ser.xonxoff = False     #disable software flow control
ser.rtscts = False     #disable hardware (RTS/CTS) flow control
ser.dsrdtr = False       #disable hardware (DSR/DTR) flow control
ser.writeTimeout = 2     #timeout for write

try: 
    ser.open()
    print("opend ttyUSB0")
except Exception:
    print("error open serial port")
    exit()

if ser.isOpen():

    try:
        # ser.flushInput() #flush input buffer, discarding all its contents
        # ser.flushOutput()#flush output buffer, aborting current output 
                 #and discard all that is in buffer

        #write data
        ser.write(b"@SERVO ON\r\n")  
        print("write data: @SERVO ON")
        # ser.write(b"@MANUAL\r\n")  
        # print("write data: @MANUAL")
        
        # A S PE ED 50
        # MO V E P, P 1 . . . . . . 按 照 35% (= 50*70) 的 速 度 从 当 前 位 置 向 P1 移 动

        ser.write(b"@ASPEED 20\r\n")  
        print("write data: @ASPEED 50")        
        time.sleep(0.5)  #give the serial port sometime to receive the data

        ser.write(b"<P1> [10,10]\r\n")
        ser.write(b"@MOVE P, P1, [1], S=10\r\n")  
        print("write data: @MOVE P1")       

        # while True:
        #     ser.write(b"@JOG X+\r\n")  
        #     print("write data: @JOG X+")
        # numOfLines = 0

        # response = ser.read(100) 

        # print("read data: " + response)

        # while True:
        #     response = ser.readline()
        #     print("read data: " + response)

        #     numOfLines = numOfLines + 1

        #     if (numOfLines >= 5):
        #         break

        # ser.close()
    except Exception:
        print("error communicating...: ")

else:
    print("cannot open serial port ")



# with serial.Serial() as ser:
#     ser.baudrate = 9600
#     ser.port = '/dev/ttyUSB0'
#     ser.bytesize=EIGHTBITS
#     ser.parity=PARITY_ODD 
#     ser.stopbits=STOPBITS_ONE
#     ser.timeout=None
#     ser.xonxoff=True
#     ser.rtscts=False
#     ser.write_timeout=None
#     ser.dsrdtr=False
#     ser.inter_byte_timeout=None
#     ser.exclusive=None


#     ser.open()
#     ser.write(b'@SERVO OFF')
#     ser.write(b'@SERVO ON')

# ser.close()     