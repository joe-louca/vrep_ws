#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool

def gripper_publisher():
    rate_hz = rospy.get_param('rate_hz')
    pub = rospy.Publisher('gripper_control', Bool, queue_size=1)
    rospy.init_node('gripper_pub_node', anonymous=True)
    rate = rospy.Rate(rate_hz)

    while not rospy.is_shutdown():
        gripper_control = rospy.get_param('gripper_control')
        msg = Bool()
        msg.data = gripper_control
        pub.publish(msg)
        rate.sleep()
        
if __name__ == '__main__':
    gripper_publisher()
