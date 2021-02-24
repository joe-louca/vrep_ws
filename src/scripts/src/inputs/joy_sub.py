#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Joy

def joy_callback(msg):
    # Convert msg into 1x8 axes list & 1x11 button list
    # -1/+1:  [ Left L/R     Left D/U     L2 In/Out    Right L/R    Right U/D    R2 In/out    D-pad L/R    D-pad D/U ]    
    ax = [msg.axes[0], msg.axes[1], msg.axes[2], msg.axes[3], msg.axes[4], msg.axes[5], msg.axes[6], msg.axes[7]]
    # 0/1:     [ X                Circle          Triangle        Square          L1              R1              Share           Options         PS Button       L3                R3           ]
    but = [msg.buttons[0], msg.buttons[1], msg.buttons[2], msg.buttons[3], msg.buttons[4], msg.buttons[5], msg.buttons[8], msg.buttons[9], msg.buttons[10], msg.buttons[11], msg.buttons[12]]    

    x = ax[0]
    y = ax[1]
    z = ax[7]
    roll = ax[3]
    pitch = ax[4]
    yaw = float(but[5] - but[4])
    rospy.set_param('ax', {'x':x, 'y':y, 'z':z, 'roll':roll, 'pitch':pitch, 'yaw':yaw})
                    
    x_press = but[0]
    rospy.set_param('but', {'x_press':x_press})
            
def main():
    # Subscribes to /joy and saves axes and button arrays to global config variables   
    rospy.init_node('joy_sub_node', anonymous=True)
    rospy.Subscriber("joy", Joy, joy_callback, queue_size=1)
    rospy.spin()
    

if __name__ == '__main__':
    main()


