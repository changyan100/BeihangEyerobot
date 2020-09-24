#!/usr/bin/env python3


import sys
import string
import gclib
import time
import galil_config
import numpy as np
import actuator_homing
import galil_ipaddress
from termcolor import colored  #for print in color
from math import cos
from math import acos
from math import pi



class actuator_control():
    def __init__(self): #set default ip address here
        ip_address = galil_ipaddress.ip_address
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
        self.g.GCommand('SH') #servo on
        print("Motor servos are on!")


    def actuator2joint_position(self, actuator_position): #q1 for scara arm...q6 for rcm small screw motor
        joint_position = np.zeros(6)
        joint_position[0] = actuator_position[0]/10000 #unit: degree  ---temp value, need update!!
        joint_position[1] = actuator_position[1]/10000 #unit: degree  ---temp value, need update!!
        joint_position[2] = actuator_position[2]/10000 #unit: mm
        joint_position[3] = actuator_position[3]/2500  #unit: degree
        
        #calculate q5 q6 from RCM structure parameters x1 x2 
        m1 = actuator_position[4]/4000  #unit: mm  for RE25 motor
        m2 = actuator_position[5]/2048  #unit: mm  for RE16 motor --need update!!
        x1 = m1+48.99  # x1=48.99 when rcm structure is in initial pose, where endtip is vertical to ground
        x2 = m2+50     # x2=50 when rcm structure is in initial pose

        rcm_b = 70 #unit: mm; distance from rcm slider joint to middle plane joint
        cos_theta = (x1**2+x2**2-rcm_b**2)/(2*x1*x2)
        q5 = acos(cos_theta)-pi/2
        q6 = x2-50

        joint_position[4] = q5  #unit: degree, rcm pivot rotate
        joint_position[5] = q6  #unit: mm , rcm insert/retract
        return joint_position


    def joint2actuator_position(self, joint_position): #q1 for scara arm...q6 for rcm small screw motor
        actuator_position = np.zeros(6)
        actuator_position[0] = joint_position[0]*10000 #unit: degree; scara arm 0 ---temp value, need update!! 
        actuator_position[1] = joint_position[1]*10000 #unit: degree; scara arm 1 ---temp value, need update!!
        actuator_position[2] = joint_position[2]*10000 #unit: mm;     Z prismatic joint
        actuator_position[3] = joint_position[3]*2500  #unit: degree; rotY joint
        
        #calculate rcm structure parameter x1 x2 from q5 q6
        q5 = joint_position[4]
        q6 = joint_position[5]
        rcm_b = 70
        x2 = q6+50
        a = 1
        b = -2*cos(q5+pi/2)*x2
        c = x2**2-rcm_b**2
        x1 = (-b+(b**2-4*a*c)**0.5)/(2*a)

        m1 = x1-48.99
        m2 = x2-50
                                        
        if x1<0:
          print(colored('[ERROR] kinematic calculate error, actuator x1 position is negative'+str(x1), 'red'))

        actuator_position[4] = m1*4000  #unit: mm;     RE25 motor, linear motion for rcm mechanism
        actuator_position[5] = m2*2048  #unit: mm;     for RE16 motor --need update!!
        return actuator_position

    def actuator2joint_velocity(self, actuator_velocity): #q1 for scara arm...q6 for rcm small screw motor
        joint_velocity = np.zeros(6)
        joint_velocity[0] = actuator_velocity[0]/10000 #unit: degree  ---temp value, need update!!
        joint_velocity[1] = actuator_velocity[1]/10000 #unit: degree  ---temp value, need update!!
        joint_velocity[2] = actuator_velocity[2]/10000 #unit: mm
        joint_velocity[3] = actuator_velocity[3]/2500  #unit: degree
        joint_velocity[5] = actuator_velocity[5]/2048  #unit: mm  ---need update!!! for RE16 motor
      
        # calculate x1_dot, to get m1_dot: RE 25 motor velocity
        joint_position = self.tell_jointposition()
        q5 = joint_position[4]
        q6 = joint_position[5]
        q5_dot = joint_velocity[4]
        q6_dot = joint_velocity[5]
        x2 = q6+50
        x2_dot = q6_dot
        rcm_b = 70
        a = 1
        b = -2*cos(q5+pi/2)*x2
        c = x2**2-rcm_b**2
        x1 = (-b+(b**2-4*a*c)**0.5)/(2*a)
        x1_dot = actuator_velocity[4]/4000

        aa = 2*x1*x2*sin(q5+pi/2) 
        bb = -2*x1*cos(q5+pi/2)*x2_dot+2*x2*x2_dot+(2*x1-2*x2*cos(q5+pi/2))*x1_dot

        q5_dot = -bb/aa
        joint_velocity[4] = q5_dot  #unit: mm;     RE25 motor, linear motion for rcm mechanism


        return joint_value

    def joint2actuator_velocity(self, joint_velocity): #q1 for scara arm...q6 for rcm small screw motor
        actuator_velocity = np.zeros(6)
        actuator_velocity[0] = joint_velocity[0]*10000 #unit: degree; scara arm 0 ---temp value, need update!! 
        actuator_velocity[1] = joint_velocity[1]*10000 #unit: degree; scara arm 1 ---temp value, need update!!
        actuator_velocity[2] = joint_velocity[2]*10000 #unit: mm;     Z prismatic joint
        actuator_velocity[3] = joint_velocity[3]*2500  #unit: degree; rotY joint
        actuator_velocity[5] = joint_velocity[5]*2048  # a6_dot = q6_dot*x #unit: mm  ---need update!!! for RE16 motor

        # calculate x1_dot, to get m1_dot: RE 25 motor velocity
        joint_position = self.tell_jointposition()
        q5 = joint_position[4]
        q6 = joint_position[5]
        q5_dot = joint_velocity[4]
        q6_dot = joint_velocity[5]
        x2 = q6+50
        x2_dot = q6_dot
        rcm_b = 70
        a = 1
        b = -2*cos(q5+pi/2)*x2
        c = x2**2-rcm_b**2
        x1 = (-b+(b**2-4*a*c)**0.5)/(2*a)

        aa = 2*x1-2*x2*cos(q5+pi/2)
        bb = -2*x1*cos(q5+pi/2)*x2_dot+2*x1*x2*sin(q5+pi/2)*q5_dot+2*x2*x2_dot

        x1_dot = -bb/aa
        actuator_velocity[4] = x1_dot*4000  #unit: mm;     RE25 motor, linear motion for rcm mechanism

        return actuator_velocity

    def actuator_jog(self, joint_vel): #joint jog motion - velocity control
        # convert joint vel to actuator vel
        actuator_vel = self.joint2actuator_velocity(joint_vel)

        # motion command sent to galil card
        # here we swap the index of joint_vel with actuator index, because galil gard axis from A->F matches to joint 6->1
        vel_cmd = 'JG'+str(actuator_vel[0])+','+str(actuator_vel[1])+','+str(actuator_vel[2])+','+str(actuator_vel[3])+','+str(actuator_vel[4])+','+str(actuator_vel[5])
        self.g.GCommand(vel_cmd) # JOG motion mode; set velocity
        self.g.GCommand('BG') #begin motion
        # time.sleep(3)
        # self.g.GCommand('ST') #stop motion
        # self.g.GCommand('MO') #motor off


    def actuator_incrementmotion(self, joint_inc):  #joint increment motion - position control
        #convert joint increment to actuator increment
        actuator_inc = self.joint2actuator_position(joint_inc)
        #send cmd to galil
        inc_cmd = 'PR'+str(actuator_inc[0])+','+str(actuator_inc[1])+','+str(actuator_inc[2])+','+str(actuator_inc[3])+','+str(actuator_inc[4])+','+str(actuator_inc[5])
        self.g.GCommand(inc_cmd)
        self.g.GCommand('BG') #begin motion


    def actuator_absolutemotion(self, joint_ap): #joint absolute motion - postion control
        #covert joint absolute position to actuator absolute postion
        actuator_ap = self.joint2actuator_position(joint_ap)
        #send cmd to galil
        ap_cmd = 'PA'+str(actuator_ap[0])+','+str(actuator_ap[1])+','+str(actuator_ap[2])+','+str(actuator_ap[3])+','+str(actuator_ap[4])+','+str(actuator_ap[5])
        self.g.GCommand(ap_cmd)
        self.g.GCommand('BG') #begin motion



    def tell_jointposition(self): #tell joints absolute positon
        # TP returns value of position encoder in string form with unit of 'cts'
        actuator_postion = np.zeros(6)
        actuator_postion[0] = float(self.g.GCommand('TP A')) #scara arm motor 1
        actuator_postion[1] = float(self.g.GCommand('TP B')) #scara arm motor 2
        actuator_postion[2] = float(self.g.GCommand('TP C')) #Z prismatic motor
        actuator_postion[3] = float(self.g.GCommand('TP D')) #roty motor
        actuator_postion[4] = float(self.g.GCommand('TP E')) #rcm motor 1 maxon RE25
        actuator_postion[5] = float(self.g.GCommand('TP F')) #rcm motor 2 maxon RCX22
        
        joint_position = self.actuator2joint_position(actuator_postion)
        return joint_position


    def tell_jointvelocity(self): #tell joints velocity
        # TP returns value of position encoder in string form with unit of 'cts'
        actuator_vel = np.zeros(6)
        actuator_vel[0] = float(self.g.GCommand('TV A')) #scara arm motor 1
        actuator_vel[1] = float(self.g.GCommand('TV B')) #scara arm motor 2
        actuator_vel[2] = float(self.g.GCommand('TV C')) #Z prismatic motor
        actuator_vel[3] = float(self.g.GCommand('TV D')) #roty motor
        actuator_vel[4] = float(self.g.GCommand('TV E')) #rcm motor 1 maxon RE25
        actuator_vel[5] = float(self.g.GCommand('TV F')) #rcm motor 2 maxon RCX22
        
        joint_vel = self.actuator2joint_velocity(actuator_vel)
        return joint_vel



    def actuator_checkrunning(self): #check if motors stopped and tell the reasons 
        actuator_sc = np.zeros(6)
        actuator_sc[0] = int(self.g.GCommand('SC A'))
        actuator_sc[1] = int(self.g.GCommand('SC B'))
        actuator_sc[2] = int(self.g.GCommand('SC C'))
        actuator_sc[3] = int(self.g.GCommand('SC D'))
        actuator_sc[4] = int(self.g.GCommand('SC E'))
        actuator_sc[5] = int(self.g.GCommand('SC F'))

        for x in range(6):
          if actuator_sc[x] != 0:
            reason = self.actuator_stopreason(actuator_sc[x])
            print(colored('[WARN] actuator' + str(x+1) +' stopped moving', 'yellow'))
            print(colored('[WARN]' + reason,  'yellow'))
        

    def actuator_stopreason(self, stopcode):
        switcher={
                0:'电机运行,独立模式',
                1:'在独立位置时,电机停止',
                2:'通过 FWD Softlimt 限位开关,减速或停止',
                3:'通过 REV Softlimt 限位开关,减速或停止',
                4:'通过 ST 命令减速或停止',
                6:'通过急停输入停止',
                7: '通过急停命令(AB)停止',
                8: '通过位置错误关断电机(OE1)来减速或停止',
                9: '寻边后停止(FE)',
                10: '回零后停止(HM)',
                11: '通过选择急停停止'
             }
        return switcher.get(stopcode,"Invalid stopcode")


    def actuator_stop(self):
        self.g.GCommand('ST') #stop motion

    def actuator_off(self):
        self.g.GCommand('MO') #stop motion

    def disconnect(self):
        '''
        shut down galil instance
        '''
        self.g.GClose()


#runs main() if example.py called from the console
if __name__ == '__main__':
    ac = actuator_control()
    joint_vel = np.zeros(6)
    joint_vel[4] = 2
    i = 0
    while 1:
      i = i+1
    #   ac.actuator_jog(joint_vel)
    #   ac.actuator_checkrunning()


