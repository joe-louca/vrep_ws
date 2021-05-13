#!/usr/bin/env python3

import rospy
import math

def delay_joy_input(joy_input):
    # Stores current pose in an array. Retrieves delayed commands. Removes old poses
    global joy_input_table
    joy_input_table.insert(0, joy_input)
    tbl_len = len(joy_input_table)
    joy_input = []
    elapsed_time = rospy.get_time()
    for i in range(tbl_len):
        if (elapsed_time > joy_input_table[i][7]+latency):
            joy_input = joy_input_table[i]
            joy_input_table = joy_input_table[:(-tbl_len+i-1)]
            break
    return joy_input
            
def gripper_toggle(gripper_control):
    if gripper_control == 0:
        gripper_control = 1
    else:
        gripper_control = 0
    return gripper_control
  
def joy_pose_goal():
    global latency
    global new_pose_table
    global curr_pose_table
    global joy_input_table

    # Parameters
    rospy.init_node('joy_control_node', anonymous=True)
    latency = rospy.get_param('latency')   
    rate_hz = rospy.get_param('rate_hz')
    rate = rospy.Rate(rate_hz)
    
    # Some variables
    button_press_current = 0
    button_press_previous = 0
    gripper_control = 0     # 0 = open, 1 = closed
    new_pose_table = []
    curr_pose_table = []
    joy_input_table = []
    
    # User instructions
    print('######### starting joystick control')
    print('######### X-axis -/+: L/R L Stick, Y-axis -/+: U/D L stick, Z-axis -/+: U/D D-Pad')
    print('######### Roll -/+: L/R R Stick, Pitch -/+: U/D R stick, Yaw -/+: L1/R1')
    print('######### Open/Close gripper: X')

    while not rospy.is_shutdown():        

        # Read params from joy subscriber        
        ax_x = rospy.get_param('ax/x')
        ax_y = rospy.get_param('ax/y')
        ax_z = rospy.get_param('ax/z')
        ax_roll = rospy.get_param('ax/roll')
        ax_pitch = rospy.get_param('ax/pitch')
        ax_yaw = rospy.get_param('ax/yaw')
        x_press = rospy.get_param('but/x_press')

        t = rospy.get_time()
        
        # Store current pose & read delayed current pose (return delay)       
        joy_input = [ax_x, ax_y, ax_z, ax_roll, ax_pitch, ax_yaw, x_press, t]
        joy_input = delay_joy_input(joy_input)
        
        if joy_input:         
            # If gripper button is pressed, toggle gripper pose
            button_press_previous = button_press_current
            button_press_current = x_press
            if button_press_previous < button_press_current:
                gripper_control = gripper_toggle(gripper_control)

            # Set the ee velocity param
            rospy.set_param('ee_vel', joy_input)
            # Set the new gripper pose param
            rospy.set_param('gripper_control', gripper_control)    

        rate.sleep()

if __name__ == '__main__':
    joy_pose_goal()


