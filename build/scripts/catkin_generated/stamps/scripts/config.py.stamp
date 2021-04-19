#!/usr/bin/env python

# Variables as ros parameters
from math import pi
from rospy import set_param

latency = 000.0 / 1000.0        # One way latency (ms to s)
rate_hz = 10                    # Loop rate (Hz)
pos_step_size = 2.0 / 1000.0    # Position step size (mm to m)
rot_step_size = 0.5 * pi/180    # Rotation step size (deg to rads)
f_threshold = 100.0             # Force sensor threshold for rigid collisions (N)
t_threshold = 5.0               # Torque sensor threshold (Nm)
friction_f_threshold = 30.0     # Force sensor threshold for frictional forces (N)
friction_t_threshold = 1.0      # Torque sensor threshold for frictional torque (Nm)
velocity = [0.1]                  # Kuka velocity

set_param('latency', latency)
set_param('rate_hz', rate_hz)
set_param('pos_step_size', pos_step_size)
set_param('rot_step_size', rot_step_size)
set_param('f_threshold',  f_threshold)
set_param('t_threshold',  t_threshold)
set_param('friction_f_threshold',  friction_f_threshold)
set_param('friction_t_threshold',  friction_t_threshold)
set_param('velocity',  velocity)

set_param('ax', {'x':0.0, 'y':0.0, 'z':0.0, 'roll':0.0, 'pitch':0.0, 'yaw':0.0})
set_param('but', {'x_press':0})
