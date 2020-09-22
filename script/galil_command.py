#!/usr/bin/env python3

'''
For better encapsulation and param sharing, I implement OOP-style.
'''
import subprocess
import sys
sys.path.append(subprocess.getoutput("rospack find galil_mixed") + '/src')
import gclib_python.gclib as gclib
import eye_op_jacobian as jacob


class g_control():
    def __init__(self, ip_address='192.168.0.42'):
        self.ip_address = ip_address
        # make an instance of the gclib python class
        self.g = gclib.py()
        # for grip close/open
        self.grip_state = False
        self.roll_his = 99999


    def connect(self):
        '''
        Connection of galil card through ip address
        '''
        # try device network connection
        try:
            self.g.GOpen(ip_address + ' --direct -s ALL')
            # self.g.GOpen('COM1 --direct')
            print(self.g.GInfo())
        except:
            print("\033[1;31;45m[WARN] Galil connection failed\033[0m")
            print("\033[1;31;45m[WARN] Stopping Galil script\033[0m")
            sys.exit()


    def g_jog(self, v1=0, v2=0, v3=0):
        '''
        function: complete JOG mode motion in cartesian space
        params: v -> velocities of theta3, theta2, d6
                g_flag -> flag that avoid repetitive definition of AC, DC
        unit: mm/s
        '''
        '''
        # TODO: convert position values to degrees; confirm axis number of last three joints of eye op robot
        # TP returns value of position encoder in string form with unit of 'cts'
        p1 = float(self.g.GCommand('TP A')) # theta3
        p2 = float(self.g.GCommand('TP B')) # theta2
        p3 = float(self.g.GCommand('TP C')) # d6
        # P.convert_to_degrees()
        '''
        # TODO： convert velocity to cts unit
        # TODO: confirm all values in the following command
        # TODO: confirm axis number; default A, B, C
        # motion command sent to galil card
        self.g.GCommand('AC 20000,20000,20000') # acceleration 20000 cts/s^2
        self.g.GCommand('DC 20000,20000,20000') # deceleration 20000 cts/s^2

        self.g.GCommand(f'JG {v1},{v2},{v3}') # JOG motion mode; set velocity
        # print(' Starting move...')
        self.g.GCommand('BG ABC') #begin motion
        self.g.GMotionComplete('ABC')
        # print(' done.')


    def disconnect(self):
        '''
        shut down galil instance
        '''
        self.g.Gclose()


    def motor_off(self, axis):
        '''
        Shut down control system, and keep position-monitor on.
        This is used for manual adjustment of robot, after which use 'SH' to activate servo.
        Input: A to F
        '''
        self.g.GCommand('MO' + axis)


    def servo_on(self, axis):
        '''
        Activate servo state.
        This alters coordinate system. Any commends of position must be resent!!!
        Input: A to F
        '''
        self.g.GCommand('SH' + axis)


    def ask_servo(self, axis):
        '''
        Request servo state
        Input: A to F
        return: off -> 0; on -> 1
        '''
        self.g.GCommand('_MO' + axis)


    def iap(self, axis, distance=5, ratio=1):
        '''
        Independent axis positioning.
        @params:
            axis -> A, B, C;
            distance unit: mm or degree
            distance: relative distance in cts
            ratio unit: 1
        '''
        distance = distance * ratio
        # TODO: convert distance to cts unit
        # distance = convert_to_cts()
        self.g.GCommand('PR' + axis + "=" + str(distance))


    def iap_a(self, axis='D', position=5, ratio=1):
        '''
        Independent axis positioning.
        @params:
            axis -> A, B, C;
            distance unit: mm or degree
            position: absolute position in cts
            ratio unit: 1
        '''
        position = position * ratio
        # TODO: convert distance to cts unit
        # distance = convert_to_cts()
        self.g.GCommand('PA' + axis + "=" + str(position))


    def grip(self, state, axis=1):
        '''
        control close&open of gripper
        @params:
            state -> 0 open; 1 close;
            axis -> hardware I/O number
        '''
        # TODO: determine hardware I/O number and revise axis
        # set bit high
        if state == 1:
            self.g.GCommand('SB' + str(axis))
        # clear bit to low
        if state == 0:
            self.g.GCommand('CB' + str(axis))


    def tp(self, axis=''):
        '''
        return position of all axis in cts unit
        @param:
            axis: '' -> return position of all available axis;
                  random combination of A to H. E.g. ABC
        '''
        return(self.g.GCommand(f'TP {axis}'))


    def out(self, axis):
        return(self.g.GCommand(f'@OUT[{axis}]'))
