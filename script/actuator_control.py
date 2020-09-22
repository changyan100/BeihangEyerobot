#!/usr/bin/env python3


import sys
import string
import gclib
import time
import galil_config
import numpy as np

class actuator_control():
    def __init__(self, ip_address='169.254.11.5'): #set default ip address here
        
        # make an instance of the gclib python class
        self.g = gclib.py()
        print('gclib version:', self.g.GVersion())

        #Connection of galil card through ip address
        try:
            self.g.GOpen(ip_address + ' --direct -s ALL')
            # self.g.GOpen('COM1 --direct')
            print(self.g.GInfo())

        except:
            print("\033[1;31;45m[WARN] Galil connection failed\033[0m")
            print("\033[1;31;45m[WARN] Stopping Galil script\033[0m")
            sys.exit()

        #load motors configuration
        galil_config.galil_config(self.g)


    def actuator2joint(self, actuator_value): #q1 for scara arm...q6 for rcm small screw motor
        joint_value = np.zeros(6)
        joint_value[0] = actuator_value[0]/10000 #unit: degree  ---temp value, need update!!
        joint_value[1] = actuator_value[1]/10000 #unit: degree  ---temp value, need update!!
        joint_value[2] = actuator_value[2]/10000 #unit: mm
        joint_value[3] = actuator_value[3]/2500  #unit: degree
        joint_value[4] = actuator_value[4]/4000  #unit: mm
        joint_value[5] = actuator_value[5]/2048  #unit: mm  for RE16 motor --need update!!
        return joint_value


    def joint2actuator(self, joint_value): #q1 for scara arm...q6 for rcm small screw motor
        actuator_value = np.zeros(6)
        actuator_value[0] = joint_value[0]*10000 #unit: degree scara arm 0 ---temp value, need update!! 
        actuator_value[1] = joint_value[1]*10000 #unit: degree scara arm 1 ---temp value, need update!!
        actuator_value[2] = joint_value[2]*10000 #unit: mm     Z prismatic joint
        actuator_value[3] = joint_value[3]*2500  #unit: degree rotY joint
        actuator_value[4] = joint_value[4]*4000  #unit: mm     RE25 motor, linear motion for rcm mechanism
        actuator_value[5] = joint_value[5]*2048  #unit: mm  for RE16 motor --need update!!
        return actuator_value

    def actuator_jog(self, joint_vel):

        # TODO: convert position values to degrees; confirm axis number of last three joints of eye op robot
        # TP returns value of position encoder in string form with unit of 'cts'
        # p1 = float(self.g.GCommand('TP A')) # theta3
        # p2 = float(self.g.GCommand('TP B')) # theta2
        # p3 = float(self.g.GCommand('TP C')) # d6
        # P.convert_to_degrees()

        # convert joint vel to actuator vel
        actuator_vel = self.joint2actuator(joint_vel)

        # motion command sent to galil card
        # here we swap the index of joint_vel with actuator index, because galil gard axis from A->F matches to joint 6->1
        vel_cmd = 'JG'+str(actuator_vel[5])+str(actuator_vel[4])+str(actuator_vel[3])+str(actuator_vel[2])+str(actuator_vel[1])+str(actuator_vel[0])
        self.g.GCommand(vel_cmd) # JOG motion mode; set velocity
        self.g.GCommand('SH') #begin motion
        self.g.GCommand('BG') #begin motion
        time.sleep(10)


    def disconnect(self):
        '''
        shut down galil instance
        '''
        self.g.Gclose()



#runs main() if example.py called from the console
if __name__ == '__main__':
    ac = actuator_control()
    joint_vel = np.zeros(6)
    joint_vel[3] = 8000
    ac.actuator_jog(joint_vel)


