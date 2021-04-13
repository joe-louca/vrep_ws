#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import TwistStamped

def falcon_callback(msg):
    x = msg.twist.linear.x
    y = msg.twist.linear.y
    z = msg.twist.linear.z
    roll = msg.twist.angular.x
    pitch = msg.twist.angular.y
    yaw = msg.twist.angular.z
    rospy.set_param('ax', {'x':x, 'y':y, 'z':z, 'roll':roll, 'pitch':pitch, 'yaw':yaw})
                    
    gripper_control = msg.header.frame_id
    if gripper_control == 'y':
        rospy.set_param('but', {'x_press':1})
    else:
        rospy.set_param('but', {'x_press':0})
            
def main():
    # Subscribes to /joy and saves axes and button arrays to global config variables   
    rospy.init_node('falcon_sub_node', anonymous=True)
    rospy.Subscriber("falcon/twist", TwistStamped, falcon_callback, queue_size=1)
    rospy.spin()
    

if __name__ == '__main__':
    main()


