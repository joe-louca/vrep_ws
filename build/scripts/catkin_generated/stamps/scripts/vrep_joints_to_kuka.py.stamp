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

def j_delay(joints):
    # Stores joint angles in an array. Retrieves delayed readings. Removes old readings
    global delayed_js

    # Store frame & timestamp
    delayed_js.insert(0, joints)
    
    tbl_len = len(delayed_js)
    elapsed_time = rospy.get_time()
    
    for i in range(tbl_len):                                # starting with the most recent  
        if (elapsed_time > delayed_js[i][7] + latency):     # compare current time vs timestamp
            joints = delayed_js[i][:6]                      # record joint space without timestamp
            delayed_js = delayed_js[:(-tbl_len+i-1)]        # remove old readings
            break
        
    return joints

def jointspace_callback():
    global jointspace

    # Read joint angles topic and add a timestamp
    t = rospy.get_time()
    j_reading = [msg.data[0], msg.data[1], msg.data[2], msg.data[3], msg.data[4], msg.data[5], msg.data[6], t]
    jointspace = j_delay(j_reading)


def main():
    global jointspace
    global j_retrieved
    rospy.init_node('joint_sub', anonymous=True)
    r = rospy.Rate(rate_hz)

    # Set up iiwa connection
    #ip ='172.31.1.148'
    #iiwa=iiwaPy(ip)
    #iiwa.setBlueOn()
    #time.sleep(2)
    #iiwa.setBlueOff()

    while not rospy.is_shutdown():
        # Get jointspace with input delay
        rospy.Subscriber("joint_angles", Float32MultiArray, jointspace_callback, queue_size=1)
        
        try:
            # Move to a joint space at a given velocity
            #iiwa.movePTPJointSpace(jointspace, velocity)
            print(jointspace)
            print(velocity)
                      
        except:
            print('an error happened')

        r.sleep()

    # Clean up    
    iiwa.close()

if __name__ == '__main__':
    global delayed_js
    global latency
    global jointspace
    global rate_hz
    global vel
    
    delayed_js = []
    latency = rospy.get_param('latency')
    rate_hz = rospy.get_param('rate_hz')
    velocity = rospy.get_param('velocity')

    main()
