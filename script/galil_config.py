#!/usr/bin/env python3


import sys
import string
import gclib

# TO DO: burn the initial config into the eeprom of Galil card, such that we dont need to run this every time

def galil_config(g):
  # g = gclib.py() #make an instance of the gclib python class
  
  ###########################################################################
  print('Start configuration')
  c = g.GCommand #alias the command callable

  #c('AB') #abort motion and program
  c('ST')
  c('MO')

  # NEED UPDATE FOR AXIS A AND B!!!!!!!!!!!!!!!
  c('MT 1,1,-2.5,-2,-1,-1;')  #'Motor Type , Analog = +-1, Pluse = +-2/2.5

  c('CE 0,0,0,0,0,0')        # Encoder type
  c('CN ,,1')            # limit switch

  #c('LD,,,0')            # limit disable

  #'set soft limits here -- need update
  c('BL ,,-300000,-250000,-280000,-51200')
  c('FL ,,300000,250000,72000,51200')

  #'off on error
  c('OE 1,1,3,1,1,1')
  #'servo loop settings
  c('KP 0,0,0,0,15,30')
  c('KD 0,0,0,0,50,100')
  c('KI 0,0,0,0,0,0')
  #'integrator limits
  c('IL 0,0,0,0,0,0')
  c('FA 0,0,0,0,0,0')
  c('FV 0,0,0,0,0,0')
  c('PL 0,0,0,0,0,0')
  #'pid loop offset only used for break.
  c('OF 0,0,0,0,0,0')

  #'torque limits IN VOLTS - see above for conversions
  #c('TL 0,0,0,0,6.6,6.6');
  #'peak torque limits
  #'todo might up the other motors to 9.99
  c('TK 0,0,0,0,9.99,9.99')
  #' error limits, if the acceleration or speed are set above system capability
  #' these might be triggered
  #' or if the system is stuck
  c('ER 1000,1000,1000,1000,200,200')

  
  c('AC 500000,500000,500000,500000,400000,400000')
  c('DC 500000,500000,500000,500000,400000,400000')
  #'Maximum velocities
  c('SP 30000,30000,30000,30000,20000,20000')

  c('MG "READY!" ')

  c('WT3000')

  print('galil_config done.')
  del c #delete the alias


 
#runs main() if example.py called from the console
if __name__ == '__main__':
  pass