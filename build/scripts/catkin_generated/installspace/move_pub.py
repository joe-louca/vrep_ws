#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64MultiArray

def main():
    rate_hz = rospy.get_param('rate_hz')
    pub = rospy.Publisher('new_pose', Float64MultiArray, queue_size=1)
    rospy.init_node('move_node', anonymous=True)
    rate = rospy.Rate(rate_hz)

    while not rospy.is_shutdown():
        try:
            new_x = rospy.get_param('new_pose/x')
            new_y = rospy.get_param('new_pose/y')
            new_z = rospy.get_param('new_pose/z')
            new_qx = rospy.get_param('new_pose/qx')
            new_qy = rospy.get_param('new_pose/qy')
            new_qz = rospy.get_param('new_pose/qz')
            new_qw = rospy.get_param('new_pose/qw')
            new_pose = [new_x, new_y, new_z, new_qx, new_qy, new_qz, new_qw]

            msg = Float64MultiArray()
            msg.data = new_pose
            pub.publish(msg)
        except:
            pass
        
        rate.sleep()
        
if __name__ == '__main__':
    main()
