# -*- coding: utf-8 -*-
"""
About the script:
An exmple on controlling KUKA iiwa robot from
Python3 using the iiwaPy3 class.

Created on 3rd-Jan-2021

@author: Mohammad SAFEEA

The robot EEF is controlled in realTime, a feedback on the joints
external torques is recieved back at Python3

"""
from iiwaPy3 import iiwaPy3
import math
import time
from datetime import datetime

start_time = datetime.now()
# returns the elapsed seconds since the start of the program
def getSecs():
   dt = datetime.now() - start_time
   secs = (dt.days * 24 * 60 * 60 + dt.seconds)  + dt.microseconds / 1000000.0
   return secs
   
ip='172.31.1.148'
#ip='localhost'
iiwa=iiwaPy3(ip)
iiwa.setBlueOn()
time.sleep(2)
iiwa.setBlueOff()
# robot control
try:

    # Move to an initial position    
    j1 = -40*math.pi/180
    j2 =  75*math.pi/180
    j3 =   0*math.pi/180
    j4 =  50*math.pi/180
    j5 = -20*math.pi/180
    j6 =  35*math.pi/180
    j7 =  0*math.pi/180
    initPos = [j1,j2,j3,j4,j5,j6,j7]
    initVel=[0.1]
    iiwa.movePTPJointSpace(initPos,initVel)
    
    counter=0
    index=2 # index of z coordinate
    w=2*math.pi/4
    theta=0
    interval= 4*math.pi
    a=50 # magnitude of motion [mm]
    
    pos=iiwa.getEEFPos()
    z0=pos[index]
    iiwa.realTime_startDirectServoCartesian()
    time.sleep(1)
    
    t0=getSecs()
    t_0=getSecs()
    while theta<interval:
        theta=w*(getSecs()-t0)
        pos[index]=z0+a*math.sin(theta)
        
        if (getSecs()-t_0)>0.002:
            exTorques = iiwa.sendEEfPositionGetExTorque(pos)
            t_0=getSecs()
            counter=counter+1
            
    print(exTorques)        
    deltat= getSecs()-t0;
    iiwa.realTime_stopDirectServoCartesian()

    # Move to an initial position    
    j1 = -40*math.pi/180
    j2 =  75*math.pi/180
    j3 =   0*math.pi/180
    j4 =  50*math.pi/180
    j5 = -20*math.pi/180
    j6 =  35*math.pi/180
    j7 =  0*math.pi/180
    jPos = [j1,j2,j3,j4,j5,j6,j7]
    vRel=[0.1]
    iiwa.movePTPJointSpace(jPos,vRel)
    
    # Print some statistical data
    print('update freq')
    print(counter/deltat)
    # In our tests it achieved 260 Hz (for full duplex command-write-feedback-read) using:
    # Windows 10, intel i7 @3.6 GHz and Python 3.6.
    # This is without proper optimizations (also an Antivirus is on - couldn't disable it, university PC, requires admin pass to diable the Antivirus)
except:
    print('an error happened')
# Close connection    
iiwa.close()


