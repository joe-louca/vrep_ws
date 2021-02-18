#!/bin/bash

cd ~/vrep_ws 
roscore &
cd ~/vrep_ws/vrep && ./coppeliaSim.sh ~/vrep_ws/src/vrep_scenes/iiwa14-robotiq2f.ttt &
rosrun joy joy_node &
rosrun scripts joy_sub.py &
rosrun scripts tf_sub.py &
rosrun scripts joy_control.py &
rosrun scripts move_pub.py &
rosrun scripts gripper_pub.py &
