#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64MultiArray

def main():
    rate_hz = rospy.get_param('rate_hz')
    pub = rospy.Publisher('ee_vel', Float64MultiArray, queue_size=1)
    rospy.init_node('move_node', anonymous=True)
    rate = rospy.Rate(rate_hz)

    while not rospy.is_shutdown():
        try:
            ee_vel = rospy.get_param('ee_vel')
            msg = Float64MultiArray()
            msg.data = new_pose[0]
            print(ee_vel)

            pub.publish(msg)
        except:
            pass
        
        rate.sleep()
        
if __name__ == '__main__':
    main()
