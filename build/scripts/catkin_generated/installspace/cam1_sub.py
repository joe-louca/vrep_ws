#!/usr/bin/env python3

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

def cam1_callback(msg):
    #t = msg.header.stamp
    t = rospy.get_time()
    
    br = CvBridge()
    img = br.imgmsg_to_cv2(msg)
    flp = cv2.flip(img, 0)
    im = cv2.cvtColor(flp, cv2.COLOR_BGR2RGB)
    cv2.imshow('cam1 stream', im)
    cv2.waitKey(1)

    #img_list = img.tolist()
    #rospy.set_param('cam1', img_list)
    
def main():   
    # Subscribes to cam img stream 
    rospy.init_node('cam1_sub_node', anonymous=True)
    rospy.Subscriber('cam1', Image, cam1_callback, queue_size=1)
    rospy.spin()
    
if __name__ == '__main__':
    main()