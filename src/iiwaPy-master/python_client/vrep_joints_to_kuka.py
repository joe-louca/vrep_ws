# -*- coding: utf-8 -*-
"""
Created on Mon Mar 26 17:03:26 2018

@author: Mohammad SAFEEA

Test script of iiwaPy class.

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
    iiwa.setBlueOn()
    time.sleep(2)
    iiwa.setBlueOff()
    
    try:
        while True:
            t = time.time()
            
            # Get joint input values
            joints = rospy.get_param('joint_angles')
            joint_cmd = [joints[0], joints[1], joints[2], joints[3], joints[4], joints[5], joints[6]]

            # Move to a joint space at a given velocity
            iiwa.movePTPJointSpace(joint_cmd, velocity)

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
    velocity = rospy.get_param('velocity')

    main()
