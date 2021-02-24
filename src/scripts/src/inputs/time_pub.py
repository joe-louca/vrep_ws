#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64

def main():
    rate_hz = rospy.get_param('rate_hz')
    pub = rospy.Publisher('elapsed_time', Float64, queue_size=1)
    rospy.init_node('elapsed_time_node', anonymous=True)
    rate = rospy.Rate(rate_hz)

    while not rospy.is_shutdown():
        elapsed_time = rospy.get_time()
        msg = Float64()
        msg.data = elapsed_time
        pub.publish(msg)
        rate.sleep()
        
if __name__ == '__main__':
    main()
