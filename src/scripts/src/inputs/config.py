#!/usr/bin/env python

# Variables as ros parameters
from math import pi
from rospy import set_param

latency = 500.0 / 1000.0        # One way latency (ms to s)
rate_hz = 50                    # Loop rate (Hz)
pos_step_size = 1.0 / 1000.0    # Position step size (mm to m)
rot_step_size = 1.0 * pi/180    # Rotation step size (deg to rads)
f_threshold = 300               # Force sensor threshold (N)
t_threshold = 5                 # Torque sensor threshold (Nm)

set_param('latency', latency)
set_param('rate_hz', rate_hz)
set_param('pos_step_size', pos_step_size)
set_param('rot_step_size', rot_step_size)
set_param('f_threshold',  f_threshold)
set_param('t_threshold',  t_threshold)

set_param('ax', {'x':0.0, 'y':0.0, 'z':0.0, 'roll':0.0, 'pitch':0.0, 'yaw':0.0})
set_param('but', {'x_press':0})
