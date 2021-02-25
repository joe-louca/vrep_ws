#!/usr/bin/env python

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

def frame_delay(img, t):
    # Stores frame and timestamps in 2 arrays. Retrieves delayed frames. Removes old poses
    global delayed_frames
    global delayed_ts

    # Store frame & timestamp
    delayed_frames.insert(0, img)
    delayed_ts.insert(0, t+latency)
    
    tbl_len = len(delayed_ts)
    frame_retrieved = False
    elapsed_time = rospy.get_time()
    for i in range(tbl_len):
        if (elapsed_time > delayed_ts[i]):
            img = delayed_frames[i]
            delayed_frames = delayed_frames[:(-tbl_len+i-1)]
            delayed_ts = delayed_ts[:(-tbl_len+i-1)]
            frame_retrieved = True
            break
    return img, frame_retrieved


def cam_callback(msg):
    t = rospy.get_time()    
    br = CvBridge()
    frame = br.imgmsg_to_cv2(msg)
    frame = cv2.flip(frame, 0)
    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    show_frame, frame_retrieved = frame_delay(frame, t)
    if frame_retrieved:
        cv2.imshow('cam2 stream', show_frame)
        cv2.waitKey(1)
    
def main():   
    # Subscribes to /tf and saves position and rotation arrays to global config variables 
    rospy.init_node('cam2_sub_node', anonymous=True)
    rospy.Subscriber('cam2', Image, cam_callback, queue_size=1)
    rospy.spin()
    
if __name__ == '__main__':
    global delayed_frames
    global delayed_ts
    global latency
    delayed_frames = []
    delayed_ts = []
    latency = rospy.get_param('latency')
    main()
