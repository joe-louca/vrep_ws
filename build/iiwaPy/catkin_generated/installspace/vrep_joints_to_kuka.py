#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Mar 26 17:03:26 2018

@author: Mohammad SAFEEA

Test script of iiwaPy class.

"""

"""
Doesnt get params from ros
Doesnt loop
"""


from iiwaPy import iiwaPy
import math
import time
import rospy
from std_msgs.msg import Float32MultiArray

def rate_sleep(t):
    dt = 1.0/rate_hz
    while True:
        if time.time() > (t + dt):
            break

def main():
    # Set up iiwa connection
    ip ='172.31.1.148'
    iiwa=iiwaPy(ip)
    time.sleep(2)
    manual_joints = [[-40*math.pi/180, +75*math.pi/180, -3*math.pi/180, +48*math.pi/180, -20*math.pi/180, +35*math.pi/180, -77*math.pi/180],
                     [-39.5*math.pi/180, +75*math.pi/180, -3*math.pi/180, +48*math.pi/180, -20*math.pi/180, +35*math.pi/180, -77*math.pi/180],
                     [-39*math.pi/180, +75*math.pi/180, -3*math.pi/180, +48*math.pi/180, -20*math.pi/180, +35*math.pi/180, -77*math.pi/180],
                     [-38.5*math.pi/180, +75*math.pi/180, -3*math.pi/180, +48*math.pi/180, -20*math.pi/180, +35*math.pi/180, -77*math.pi/180],
                     [-38*math.pi/180, +75*math.pi/180, -3*math.pi/180, +48*math.pi/180, -20*math.pi/180, +35*math.pi/180, -77*math.pi/180],
                     [-37.5*math.pi/180, +75*math.pi/180, -3*math.pi/180, +48*math.pi/180, -20*math.pi/180, +35*math.pi/180, -77*math.pi/180],
                     [-37*math.pi/180, +75*math.pi/180, -3*math.pi/180, +48*math.pi/180, -20*math.pi/180, +35*math.pi/180, -77*math.pi/180],
                     [-36.5*math.pi/180, +75*math.pi/180, -3*math.pi/180, +48*math.pi/180, -20*math.pi/180, +35*math.pi/180, -77*math.pi/180],
                     [-36*math.pi/180, +75*math.pi/180, -3*math.pi/180, +48*math.pi/180, -20*math.pi/180, +35*math.pi/180, -77*math.pi/180],
                     [-35*math.pi/180, +75*math.pi/180, -3*math.pi/180, +48*math.pi/180, -20*math.pi/180, +35*math.pi/180, -77*math.pi/180],
                     [-34*math.pi/180, +75*math.pi/180, -3*math.pi/180, +48*math.pi/180, -20*math.pi/180, +35*math.pi/180, -77*math.pi/180],
                     [-33*math.pi/180, +75*math.pi/180, -3*math.pi/180, +48*math.pi/180, -20*math.pi/180, +35*math.pi/180, -77*math.pi/180],
                     [-32*math.pi/180, +75*math.pi/180, -3*math.pi/180, +48*math.pi/180, -20*math.pi/180, +35*math.pi/180, -77*math.pi/180],
                     [-31*math.pi/180, +75*math.pi/180, -3*math.pi/180, +48*math.pi/180, -20*math.pi/180, +35*math.pi/180, -77*math.pi/180],
                     [-30*math.pi/180, +75*math.pi/180, -3*math.pi/180, +48*math.pi/180, -20*math.pi/180, +35*math.pi/180, -77*math.pi/180]]
    
    try:
        #while True:
        for i in range(15):
            t = time.time()
            
            # Get joint input values
            joints = rospy.get_param('joint_angles')
            #joint_cmd = [joints[0], joints[1], joints[2], joints[3], joints[4], joints[5], joints[6]]
            joint_cmd = manual_joints[i]
            
            # Testing
            print(joint_cmd)
            print(velocity)
            
            # Move to a joint space at a given velocity
            #iiwa.movePTPJointSpace(joint_cmd, velocity)

            # Run loop at given rate
            rate_sleep(t)
            
    except KeyboardInterrupt:
        print('exiting kuka control')

    # Clean up    
    iiwa.close()

if __name__ == '__main__':
    global rate_hz
    global vel

    rate_hz = rospy.get_param('rate_hz')
    velocity = [rospy.get_param('velocity')]
    rate_hz = 10
    velocity = [0.1]
    
    main()
