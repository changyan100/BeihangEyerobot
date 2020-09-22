#!/usr/bin/env python3


import sys
import string
import gclib


def galil_config(g):
  # g = gclib.py() #make an instance of the gclib python class
  
  ###########################################################################
  print('Start configuration')
  c = g.GCommand #alias the command callable

  # c('AB') #abort motion and program
  c('MT -1,-1,-2,-2.5')  #'Motor Type , Analog = +-1, Pluse = +-2/2.5

  c('CE 0,0,0,0')        # Encoder type
  c('CN,,,1')            # limit switch

  #c('LD,,,0')            # limit disable

  #'set soft limits here
  c('BL -51200,-280000,-250000,-300000')
  c('FL 51200,72000,250000,300000')

  #'off on error
  c('OE1,1,1,3')
  #'servo loop settings
  c('KP90,15,0,0')
  c('KD480,50,0,0')
  c('KI0,0,0,0')
  #'integrator limits
  c('IL0,0,0,0')
  c('FA10,0,0,0')
  c('FV10,0,0,0')
  c('PL0,0,0,0')
  #'pid loop offset only used for break.
  c('OF0,0,0,0')

  #'torque limits IN VOLTS - see above for conversions
  c('TL6,6');
  #'peak torque limits
  #'todo might up the other motors to 9.99
  c('TK9.999,9.999')
  #' error limits, if the acceleration or speed are set above system capability
  #' these might be triggered
  #' or if the system is stuck
  c('ER200,200,1000,1000')

  
  c('AC300000,400000,500000,500000')
  c('DC300000,400000,500000,500000')
  #'Maximum velocities
  c('SP30000,40000,50000,50000')

  c('MG "READY!" ')

  c('WT3000')

  print('galil_config done.')
  del c #delete the alias


 
#runs main() if example.py called from the console
if __name__ == '__main__':
  pass