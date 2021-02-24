#!/usr/bin/env python

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

def delay_cam(image):
    # Stores current pose in an array. Retrieves delayed commands. Removes old poses
    global joy_input_table
    joy_input_table.insert(0, joy_input)
    tbl_len = len(joy_input_table)
    joy_input = []
    elapsed_time = rospy.get_time()
    for i in range(tbl_len):
        if (elapsed_time > joy_input_table[i][7]+latency):
            joy_input = joy_input_table[i]
            joy_input_table = joy_input_table[:(-tbl_len+i-1)]
            break
    return joy_input


def cam1_callback(msg):
    br = CvBridge()
    image = br.imgmsg_to_cv2(msg)
    resize = cv2.resize(image, (240,240))
    #flp = cv2.flip(img, 0)
    #im = cv2.cvtColor(flp, cv2.COLOR_BGR2RGB)
    cv2.imshow("cam1", resize)
    cv2.waitKey(1)
    #rospy.set_param('cam1', msg)

def cam_viewer():
    rospy.init_node('joy_control_node', anonymous=True)
    rate_hz = rospy.get_param('rate_hz')
    rate = rospy.Rate(rate_hz)
    
    # Parameters
    global latency
    latency = rospy.get_param('latency')   
    
    # Some variables
    global cam1_table
    cam1_table = []

    while not rospy.is_shutdown():        

        cam1_img = rospy.get_param('cam1/image')   

    
if __name__ == '__main__':
    cam_viewer()
