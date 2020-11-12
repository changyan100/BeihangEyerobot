#!/usr/bin/env python3

import rospy
from std_msgs.msg import String

import struct


class pedal():
    """docstring for pedal"""
    def __init__(self):
        self.infile_path = "/dev/input/event7"
        self.EVENT_SIZE = struct.calcsize("llHHI")
        print("1")
        self.file = open(self.infile_path, "rb")
        print("2")
        self.event = self.file.read(self.EVENT_SIZE)
        print("3")

        self.pedal_reading = "null"

    def pedal_detect(self):
        (tv_sec, tv_usec, type, code, value) = struct.unpack("llHHI", self.event)
        self.event = self.file.read(self.EVENT_SIZE)
        value = struct.unpack("llHHI", self.event)
        pedal = value[3]
        flag = value[2]
        release = value[4]
        
        if int(flag) == 1:
            if int(release) == 0:
                self.pedal_reading = "null"
            else:
                if int(pedal) == 38:
                   self.pedal_reading = "left"
                elif int(pedal) == 19:
                    self.pedal_reading = "right"
                else:
                    pass
        else:
            pass

        return self.pedal_reading



if __name__ == '__main__':

    try:
        pub = rospy.Publisher('pedal_value', String, queue_size=10)
        rospy.init_node('pedal', anonymous=True)
        rate = rospy.Rate(1000) # 1 kHz
        print("I am in program")
        pd = pedal()
        
        while not rospy.is_shutdown():
            print("I am in while")
            pedal_output = pd.pedal_detect()
            # rospy.loginfo(pedal_output)
            pub.publish(pedal_output)
            rate.sleep()
            print(pedal_output)

    except rospy.ROSInterruptException:
        pass

