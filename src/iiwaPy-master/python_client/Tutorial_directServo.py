# -*- coding: utf-8 -*-
"""
Created on Mon Mar 26 17:03:26 2018

@author: Mohammad SAFEEA

Test script of iiwaPy class.

"""
from iiwaPy import iiwaPy
import math
import time
   
ip='172.31.1.148'
#ip='localhost'
iiwa=iiwaPy(ip)
iiwa.setBlueOn()
time.sleep(2)
iiwa.setBlueOff()

# read some data from the robot
try:
    # Move to an initial position    
    initPos=[-40*math.pi/180,math.pi*60/180,0,math.pi*90/180,0,math.pi*40/180,0]
    initVel=[0.1]
    iiwa.movePTPJointSpace(initPos,initVel)
    
    
except:
    print('an error happened')
    
iiwa.close()
print('update freq')
#print(counter/deltat)

