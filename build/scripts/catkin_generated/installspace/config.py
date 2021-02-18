#!/usr/bin/env python3

# Variables as ros parameters
from math import pi
from rospy import set_param

rate_hz = 10            # Loop rate (Hz)
pos_step_size = 0.01    # Position step size (m)
rot_step_size = pi/180  # Rotation step size (rads)

set_param('/rate_hz', rate_hz)
set_param('/pos_step_size', pos_step_size)
set_param('/rot_step_size', rot_step_size)
