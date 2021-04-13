#!/usr/bin/env python3

import rospy
import cv2
from std_msgs.msg import Float32MultiArray

def ft_delay(ft):
    # Stores ft readings in an array. Retrieves delayed readings. Removes old readings
    global delayed_fts

    # Store frame & timestamp
    delayed_fts.insert(0, ft)
    
    tbl_len = len(delayed_fts)
    ft_retrieved = False
    elapsed_time = rospy.get_time()
    for i in range(tbl_len):
        if (elapsed_time > delayed_fts[i][6] + latency):
            ft = delayed_fts[i]
            delayed_fts = delayed_fts[:(-tbl_len+i-1)]
            ft_retrieved = True
            break
        
    return ft, ft_retrieved


def ft_callback(msg):
    t = rospy.get_time()
    ft_reading = [msg.data[0], msg.data[1], msg.data[2], msg.data[3], msg.data[4], msg.data[5], t]
    ft, ft_retrieved = ft_delay(ft_reading)
    # If above threshold, send vibration feedback
    F = ft[0]*ft[0] + ft[1]*ft[1] + ft[2]*ft[2]
    T = ft[3]*ft[3] + ft[4]*ft[4] + ft[5]*ft[5]
    if F > f_threshold or T > t_threshold:
        vib_fb = True
    else:
        vib_fb = False

    # Update param    
    rospy.set_param('vib_fb', vib_fb)
    rospy.set_param('ft', ft)

    
def main():   
    # Subscribes to /tf and saves position and rotation arrays to global config variables 
    rospy.init_node('ft_sub_node', anonymous=True)
    rospy.Subscriber('ft_sensor', Float32MultiArray, ft_callback, queue_size=1)
    rospy.spin()

  
if __name__ == '__main__':
    global delayed_fts
    global latency
    delayed_fts = []
    latency = rospy.get_param('latency')
    f_threshold = rospy.get_param('f_threshold')
    t_threshold = rospy.get_param('t_threshold')
    main()
