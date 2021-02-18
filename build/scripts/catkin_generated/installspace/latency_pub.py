#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int32

def main():
    rate_hz = rospy.get_param('rate_hz')
    pub = rospy.Publisher('latency', Int32, queue_size=1)
    rospy.init_node('latency_node', anonymous=True)
    rate = rospy.Rate(rate_hz)

    while not rospy.is_shutdown():
        latency = rospy.get_param('latency')
        msg = Int32()
        msg.data = latency
        pub.publish(msg)
        rate.sleep()
        
if __name__ == '__main__':
    main()
