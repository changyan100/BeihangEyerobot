#! /usr/bin/env python3

import device_ip

import hyperion
import asyncio
import numpy as np
import rospy
from BeihangEyerobot.msg import floatlist
from termcolor import colored  #for print in color
import math


class fbg_sensor():
    """docstring for fbg_sensor"""
    def __init__(self):
        # super(fbg_sensor, self).__init__()
        instrument_ip = device_ip.hyperion_integrator_ip
        self.loop = asyncio.get_event_loop()
        self.queue = asyncio.Queue(maxsize=5, loop=self.loop)
        self.serial_numbers = []

        # create the streamer object instance
        self.peaks_streamer = hyperion.HCommTCPPeaksStreamer(instrument_ip, self.loop, self.queue)
        #set fiber num here
        self.fibernum = 3
        
        #set force dimension here
        self.force_dimension = 2
        
        self.calib_matrics = np.zeros([2,3])
        # update calibration matrics here
        self.calib_matrics[0,0] = 269.1142
        self.calib_matrics[0,1] = -6.936
        self.calib_matrics[0,2] = -262.16
        self.calib_matrics[1,0] = 186.4413
        self.calib_matrics[1,1] = -534.44
        self.calib_matrics[1,2] = 347.99

        self.initcount = 0
        self.rebias_wavelength = np.zeros(self.fibernum)

        self.pub1 = rospy.Publisher('fbg_force', floatlist, queue_size=10)
        self.pub2 = rospy.Publisher('fbg_peaks', floatlist, queue_size=10)
        


# define a coroutine that pulls data out of the streaming queue and processes it.

    async def get_data(self,loop):

        wave_init_pool = np.zeros([self.fibernum,1000]) 
        init_flag = True
        force_pub = [0,0,0] #[force_norm, Fx, Fy]
        delt_w = np.zeros(self.fibernum)

        # while True:
        while not rospy.is_shutdown():

            peak_data = await self.queue.get()
            self.queue.task_done()
            rate = rospy.Rate(1000) # 1 kHz
            if peak_data['data']:
                self.serial_numbers.append(peak_data['data'].header.serial_number)
                rawdata = peak_data['data'].data
                if len(rawdata)!=self.fibernum:
                    print(colored('[ERROR] Some fibers are broken, the active fiber number is '+len(rawdata), 'red'))
                else:
                    if self.initcount<1000:
                        for x in range(0,self.fibernum):
                            wave_init_pool[x, self.initcount] = rawdata[x]
                        self.initcount = self.initcount+1
                    elif init_flag == True:
                        self.rebias_wavelength = np.mean(wave_init_pool,1)
                        print("finished collecting rebias wavelength: ", self.rebias_wavelength)
                        init_flag = False
                    else:
                        for xx in range(0,self.fibernum):
                            delt_w[xx] = rawdata[xx]-self.rebias_wavelength[xx]
                        delt_w_ave = np.sum(delt_w)/self.fibernum
                        delt_s = delt_w - delt_w_ave
                        
                        force = np.dot(self.calib_matrics, delt_s)
                        force = force*1000 #unit: mN
                        force_norm = math.sqrt(force[0]**2 + force[1]**2)
                        # print("force_norm = ", force_norm)
                        # print("delt_s = ", delt_s)
                        force_pub = [force[0], force[1], force_norm]
                self.pub1.publish(force_pub)
                self.pub2.publish(peak_data['data'].data)
                rate.sleep()
            else:
                # If the queue returns None, then the streamer has stopped.
                break
        print("while break!")
         
        loop.stop() 
        print("loop stop!")
        
        loop.run_forever()
        loop.close()



if __name__ == '__main__':

    fb = fbg_sensor()
    rospy.init_node('fbg_force', anonymous=True)
    
    fb.loop.create_task(fb.get_data(fb.loop))

# streaming_time = 20 # seconds

#Call stop_streaming after the specified amount of time.

    # loop.call_later(streaming_time, peaks_streamer.stop_streaming)
    print("loop start!")
    fb.loop.run_until_complete(fb.peaks_streamer.stream_data())


# assert (np.diff(np.array(serial_numbers)) == 1).all()
