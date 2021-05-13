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
        if (elapsed_time > delayed_js[i][6] + latency):     # compare current time vs timestamp
            joints = delayed_js[i][:6]                      # record joint space without timestamp
            delayed_js = delayed_js[:(-tbl_len+i-1)]        # remove old readings
            break
        
    return joints

def jointspace_callback(msg):
    # Read joint angles topic and add a timestamp
    t = rospy.get_time()
    j_reading = [msg.data[0], msg.data[1], msg.data[2], msg.data[3], msg.data[4], msg.data[5], t]

    # Store joint reading and retrieve delayed joint angles
    jointspace = j_delay(j_reading)

    # Set param to variable
    rospy.set_param('joint_angles', jointspace)  


def main():
    rospy.init_node('joint_sub', anonymous=True)
    rospy.Subscriber("joint_angles", Float32MultiArray, jointspace_callback, queue_size=1)
    rospy.spin()


if __name__ == '__main__':
    global delayed_js
    global latency
    delayed_js = []
    latency = rospy.get_param('latency')
    
    main()
