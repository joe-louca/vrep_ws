#!/usr/bin/env python

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
            joints = delayed_js[i][:7]                      # record joint space without timestamp
            delayed_js = delayed_js[:(-tbl_len+i-1)]        # remove old readings
            break
        
    return joints

def jointspace_callback(msg):
    global joint_now
    
    # Read joint angles topic and add a timestamp
    t = rospy.get_time()
    j_reading = [msg.data[0], msg.data[1], msg.data[2], msg.data[3], msg.data[4], msg.data[5], msg.data[6], t]

    # Store joint reading and retrieve delayed joint angles
    joint_next = j_delay(j_reading)

    # If first reading, just set now & next positions the same
    if joint_now == []:
        joint_now = joint_next

    # Calculate joint vel command based on difference between two points & refresh rate
    joint_vel = [(m-n)*rate_hz for m,n in zip(joint_next,joint_now)] # rads sec-1
        
    # Set param to variable
    rospy.set_param('joint_angles', joint_next)
    rospy.set_param('joint_vel', joint_vel)  

    # Reset for next frame
    joint_now = joint_next

def main():
    rospy.init_node('joint_sub', anonymous=True)
    rospy.Subscriber("joint_angles", Float32MultiArray, jointspace_callback, queue_size=1)
    rospy.spin()


if __name__ == '__main__':
    global delayed_js
    global latency
    global rate_hz
    global joint_now
    delayed_js = []
    latency = rospy.get_param('latency')
    rate_hz = rospy.get_param('rate_hz')    
    joint_now = []
    
    main()
