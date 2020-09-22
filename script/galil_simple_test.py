#!/usr/bin/env python3


import sys
import string
import gclib
import galil_config

def main():
  g = gclib.py() #make an instance of the gclib python class
  
  try:
    print('gclib version:', g.GVersion())

    ###########################################################################
    #  Connect
    ###########################################################################
    g.GOpen('169.254.11.5 --direct -s ALL')
    #g.GOpen('COM1 --direct')
    print(g.GInfo())
        
    ###########################################################################
    # Programs
    ###########################################################################
    # g.GProgramDownloadFile('./dmc/RobotConfig.dmc', '')
    # print(' Uploaded program:\n%s' % g.GProgramUpload())
    # # g.GCommand('ZAX=0')
    # g.GCommand('XQ #LOADSET, 1') #execute the code
    # g.GSleep(10) #wait a brief interval for the code to complete.

    galil_config.galil_config(g)

    ###########################################################################
    # Misc
    ###########################################################################
    print('Motion Complete')
    c = g.GCommand #alias the command callable
    # c('AB') #abort motion and program
    # c('MT ,,-2')
    # c('CE ,,0')
    c('WT200')
    c('DPC=0')
    c('MO') #turn off all motors
    c('SHC') #servo C
    # c('SPC=30000') #speead, 1000 cts/sec
    # c('PAC=200000') #relative move, 3000 cts
    c('AC 100000,100000,100000') # acceleration 20000 cts/s^2
    c('DC 100000,100000,100000') # deceleration 20000 cts/s^2

    c('JG 0, 0, 30000, 0') # JOG motion mode; set velocity
    print(' Starting move...')
    c('BGC') #begin motion
    # c('AMC')
    # c('WT250')
    # c('PAC=0') #relative move, 3000 cts
    # print(' Starting move...')
    # c('BGC') #begin motion
    # c('AMC')
    # c('WT250')
    g.GMotionComplete('C')
    print(' done.')
    del c #delete the alias

  ###########################################################################
  # except handler
  ###########################################################################  
  except gclib.GclibError as e:
    print('Unexpected GclibError:', e)
  
  finally:
    g.GClose() #don't forget to close connections!
  
  return
  
 
#runs main() if example.py called from the console
if __name__ == '__main__':
  main()
