#!/usr/bin/env python3


import gclib
import sys
import string
import galil_ipaddress

def homing():
  ip_address=galil_ipaddress.ip_address
    
    # make an instance of the gclib python class
  g = gclib.py()
#    print('gclib version:', self.g.GVersion())

    #Connection of galil card through ip address
  try:
    g.GOpen(ip_address + ' --direct -s ALL')
        # self.g.GOpen('COM1 --direct')
    print(g.GInfo())

  except:
    print("\033[1;31;45m[WARN] Galil connection failed\033[0m")
    print("\033[1;31;45m[WARN] Stopping Galil script\033[0m")
    sys.exit()

  g.GCommand('ST') #stop motion
  g.GCommand('MO') #motor 
  g.GCommand('DP 0,0,0,0,0,0') #set main encoder position as 0
  g.GCommand('DE 0,0,0,0,0,0') #set dual encoder position as 0, for stepper motor mode, need set DE as 0 for homing

  g.GClose()

if __name__ == '__main__':
	homing()
    