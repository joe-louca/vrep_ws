#!/usr/bin/env python3

# Variables as ros parameters
from math import pi
from rospy import set_param

latency = 100.0 / 1000.0        # One way latency (ms to s)
rate_hz = 30                    # Loop rate (Hz)
pos_step_size = 1.0 / 1000.0    # Position step size (mm to m)
rot_step_size = 0.5 * pi/180    # Rotation step size (deg torads)

set_param('latency', latency)
set_param('rate_hz', rate_hz)
set_param('pos_step_size', pos_step_size)
set_param('rot_step_size', rot_step_size)
