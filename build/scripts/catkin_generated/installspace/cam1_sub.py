#!/usr/bin/env python3

import rospy
import cam1
from sensor_msgs.msg import Image


def tf_callback(msg):
    rospy.set_param('cam1', msg)
    
def main():
    # Subscribes to /tf and saves position and rotation arrays to global config variables 
    rospy.init_node('cam1_logger_node', anonymous=True)
    rospy.Subscriber("cam1", Image, cam1_callback, queue_size=1)
    rospy.spin()
    
if __name__ == '__main__':
    main()
