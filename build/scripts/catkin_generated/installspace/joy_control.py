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

def delay_curr_pose(curr_pose):
    # Stores current pose in an array. Retrieves delayed commands. Removes old poses
    global curr_pose_table
    curr_pose_table.insert(0, curr_pose)
    tbl_len = len(curr_pose_table)
    curr_pose = []
    elapsed_time = rospy.get_time()
    for i in range(tbl_len):
        if (elapsed_time > curr_pose_table[i][8]+latency):
            curr_pose = curr_pose_table[i]
            curr_pose_table = curr_pose_table[:(-tbl_len+i-1)]
            break
    return curr_pose

def delay_new_pose(new_pose):
    # Stores new pose in an array. Retrieves delayed commands. Removes old commands
    global new_pose_table
    new_pose_table.insert(0, new_pose)
    tbl_len = len(new_pose_table)
    new_pose = []
    elapsed_time = rospy.get_time()
    for i in range(tbl_len):
        if (elapsed_time > new_pose_table[i][8]+latency):
            new_pose = new_pose_table[i]
            new_pose_table = new_pose_table[:(-tbl_len+i-1)]
            break
    return new_pose
            
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

def EasyAngle(ang):
    while ang >= 2*math.pi:
        ang -= 2*math.pi
    while ang < 0:
        ang += 2*math.pi
    return ang

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
    rate_hz = rospy.get_param('rate_hz')
    rate = rospy.Rate(rate_hz)
    
    # Parameters
    pos_step_size = rospy.get_param('pos_step_size')   
    rot_step_size = rospy.get_param('rot_step_size')
    global latency
    latency = rospy.get_param('latency')   

    # Some variables
    button_press_current = 0
    button_press_previous = 0
    gripper_control = 0     # 0 = open, 1 = closed
    global new_pose_table
    new_pose_table = []
    global curr_pose_table
    curr_pose_table = []
    global joy_input_table
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

        # Get current end effector pose from VREP
        curr_x = rospy.get_param('curr_pos/x')
        curr_y = rospy.get_param('curr_pos/y')
        curr_z = rospy.get_param('curr_pos/z')
        curr_qx = rospy.get_param('curr_rot/qx')
        curr_qy = rospy.get_param('curr_rot/qy')
        curr_qz = rospy.get_param('curr_rot/qz')
        curr_qw = rospy.get_param('curr_rot/qw')

        # Store current pose & read delayed current pose (return delay)
        t = rospy.get_time()
        curr_pose = [curr_x, curr_y, curr_z, curr_qx, curr_qy, curr_qz, curr_qw, gripper_control, t]

        if curr_pose:
            # Construct useful lists
            curr_pos = [curr_pose[0], curr_pose[1], curr_pose[2]]
            curr_rot = [curr_pose[3], curr_pose[4], curr_pose[5], curr_pose[6]]

            # Store current joy input & read delayed command (outward delay)
            joy_input = [ax_x, ax_y, ax_z, ax_roll, ax_pitch, ax_yaw, x_press, t]
            joy_input = delay_joy_input(joy_input)
            if joy_input:
                ax = [joy_input[0], joy_input[1], joy_input[2], joy_input[3], joy_input[4], joy_input[5]]
                but = [joy_input[6]]
            
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
                #new_qx = EasyAngle(new_qx)
                #new_qy = EasyAngle(new_qy)
                #new_qz = EasyAngle(new_qz)
                #new_qw = EasyAngle(new_qw)
                        
                # If gripper button is pressed, toggle gripper pose
                button_press_previous = button_press_current
                button_press_current = but[0]
                if button_press_previous < button_press_current:
                    gripper_control = gripper_toggle(gripper_control)

                # Store pose & retrieve delayed command    
                t = rospy.get_time()
                new_pose = [new_x, new_y, new_z, new_qx, new_qy, new_qz, new_qw, gripper_control, t]
                
                if new_pose:
                    # Set the new pose param (delayed)
                    rospy.set_param('new_pose', {'x':new_pose[0], 'y':new_pose[1], 'z':new_pose[2],
                                                 'qx': new_pose[3], 'qy':new_pose[4], 'qz':new_pose[5], 'qw':new_pose[6]})
                    # Set the new gripper pose param
                    rospy.set_param('gripper_control', new_pose[7])    

        rate.sleep()

if __name__ == '__main__':
    joy_pose_goal()


