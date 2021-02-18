#!/usr/bin/env python3

import rospy
from tf2_msgs.msg import TFMessage


def tf_callback(msg):
    x = msg.transforms[0].transform.translation.x
    y = msg.transforms[0].transform.translation.y
    z = msg.transforms[0].transform.translation.z
    rospy.set_param('curr_pos', {'x':x, 'y':y, 'z':z})

    qx = msg.transforms[0].transform.rotation.x
    qy = msg.transforms[0].transform.rotation.y
    qz = msg.transforms[0].transform.rotation.z
    qw = msg.transforms[0].transform.rotation.w
    rospy.set_param('curr_rot', {'qx':qx, 'qy':qy, 'qz':qz, 'qw':qw})

    
def main():
    # Subscribes to /tf and saves position and rotation arrays to global config variables 
    rospy.init_node('tf_logger_node', anonymous=True)
    rospy.Subscriber("tf", TFMessage, tf_callback, queue_size=1)
    rospy.spin()
    
if __name__ == '__main__':
    main()
