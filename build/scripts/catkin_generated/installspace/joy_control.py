#!/usr/bin/env python3

import config
import rospy
import math

def ax_or_but_press(ax, but):
    # Function to check for any joystick input
    press_total = 0.0
    for i in range(len(ax)):
        press_total += abs(ax[i])
    for i in range(len(but)):
        press_total += abs(but[i])
    if press_total > 0:
        return True
    else:
        return False

def EulerToQuaternion(roll, pitch, yaw):
    qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
    qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
    qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
    qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
    return qx, qy, qz, qw      

def QuarternionToEuler(qx, qy, qz, qw):
    t0 = +2.0 * (qw * qx + qy * qz)
    t1 = +1.0 - 2.0 * (qx * qx + qy * qy)
    roll = math.atan2(t0, t1)
    t2 = +2.0 * (qw * qy - qz * qx)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch = math.asin(t2)
    t3 = +2.0 * (qw * qz + qx * qy)
    t4 = +1.0 - 2.0 * (qy * qy + qz * qz)
    yaw = math.atan2(t3, t4)
    return roll, pitch, yaw

def gripper_toggle(gripper_control):
    if gripper_control == 0:
        gripper_control = 1
    else:
        gripper_control = 0
    return gripper_control
  
def joy_pose_goal():
    rospy.init_node('joy_control_node', anonymous=True)

    # Parameters
    pos_step_size = rospy.get_param('pos_step_size')   
    rot_step_size = rospy.get_param('rot_step_size')
    latency = rospy.get_param('latency')   

    # Some variables
    button_press_current = 0
    button_press_previous = 0
    gripper_control = 0     # 0 = open, 1 = closed

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

        # Get current end effector pose from VREP
        curr_x = rospy.get_param('curr_pos/x')
        curr_y = rospy.get_param('curr_pos/y')
        curr_z = rospy.get_param('curr_pos/z')
        curr_qx = rospy.get_param('curr_rot/qx')
        curr_qy = rospy.get_param('curr_rot/qy')
        curr_qz = rospy.get_param('curr_rot/qz')
        curr_qw = rospy.get_param('curr_rot/qw')
        
        # Construct useful arrays
        ax = [ax_x, ax_y, ax_z, ax_roll, ax_pitch, ax_yaw]
        but = [x_press]
        curr_pos = [curr_x, curr_y, curr_z]
        curr_rot = [curr_qx, curr_qy, curr_qz, curr_qw]

        # Determine if command received (true/false)
        command_received = ax_or_but_press(ax, but)
        
        ## Calculate new end effector goal pose, based on the current pose & command input:
        # Update pose (translation) & save to global config var
        new_x = curr_pos[0] + pos_step_size * ax[0]
        new_y = curr_pos[1] + pos_step_size * ax[1]
        new_z = curr_pos[2] + pos_step_size * ax[2]        

        # Update pose (rotation) & save to global config var
        curr_roll, curr_pitch, curr_yaw = QuarternionToEuler(curr_rot[0], curr_rot[1], curr_rot[2], curr_rot[3])
                  
        new_roll = curr_roll + rot_step_size * ax[3]
        new_pitch = curr_pitch + rot_step_size * ax[4]
        new_yaw = curr_yaw + rot_step_size * ax[5] 

        new_qx, new_qy, new_qz, new_qw = EulerToQuaternion(new_roll, new_pitch, new_yaw)                    

        elapsed_time = rospy.get_time()
        
        # Set the new pose param
        rospy.set_param('new_pose', {'x':new_x, 'y':new_y, 'z':new_z, 'qx': new_qx, 'qy':new_qy, 'qz':new_qz, 'qw':new_qw, 't':elapsed_time, 'l':latency})
                    
        # If gripper button is pressed, toggle gripper pose
        button_press_previous = button_press_current
        button_press_current = but[0]
        if button_press_previous < button_press_current:
            gripper_control = gripper_toggle(gripper_control)

        # Set the new gripper pose param
        rospy.set_param('gripper_control', gripper_control)    

if __name__ == '__main__':
    joy_pose_goal()


