#!/usr/bin/env python3

import struct

infile_path = "/dev/input/event7"
EVENT_SIZE = struct.calcsize("llHHI")
file = open(infile_path, "rb")
event = file.read(EVENT_SIZE)



while event:
    # print(struct.unpack("llHHI", event))
    (tv_sec, tv_usec, type, code, value) = struct.unpack("llHHI", event)
    event = file.read(EVENT_SIZE)
    value = struct.unpack("llHHI", event)
    pedal = value[3]
    flag = value[2]
    release = value[4]
    if int(flag) == 1:
        if int(release) == 0:
            print("pedal = none")
        else:
            if int(pedal) == 38:
               print("pedal = left")
            elif int(pedal) == 19:
                print("pedal = right")
            else:
           	    pass
            
    else:
        pass
#     pedal = struct.unpack("llHHI", event)

#     print('pedal = ', pedal)