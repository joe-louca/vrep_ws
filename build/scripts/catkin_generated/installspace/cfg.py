#!/usr/bin/env python3

# Store global variables to pass between modules
import rospy
from math import pi

def variables():
    global rate_hz, pos_step_size, rot_step_size, ax, but, pos, rot, new_pos, new_rot
    rate_hz = 10            # Loop rate (Hz)
    pos_step_size = 0.01    # Position step size (m)
    rot_step_size = pi/180  # Rotation step size (rads)
    ax= [] * 8              # Joystick axes
    but = [] * 11           # Joystick buttons
    pos = [] * 3            # End effector position
    rot = [] * 4            # End effector rotation
    new_pos = [] * 3        # Target position 
    new_rot = [] * 4        # Target rotation

