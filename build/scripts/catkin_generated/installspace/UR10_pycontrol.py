#!/usr/bin/env python3
import roslib
#roslib.load_manifest('ur_tutorial')
import rospy
import math
from trajectory_msgs.msg import JointTrajectory,JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryGoal, FollowJointTrajectoryAction
from actionlib import SimpleActionClient

from sensor_msgs.msg import JointState


rospy.sleep(5)

rospy.init_node("simple_traj")
client = SimpleActionClient("/follow_joint_trajectory", FollowJointTrajectoryAction) # real arm

UR10joints = rospy.Subscriber('/joint_states', JointState)

print "waiting to connect to client..."
client.wait_for_server()
print "connected to client! "

print "waiting to connect to joint subscriber..."
client.wait_for_server()
print "connected to joint subscriber! "

#print(UR10joints) # work this out!


# Set up goal variable
g = FollowJointTrajectoryGoal()
g.trajectory = JointTrajectory()
g.trajectory.joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']


# Set joint positions, p0, to reach:
# Location [0.55 0.25 0.2 ]
# Orientation:
#[-0.99950656  0.          0.03141076]
#[ 0.          1.          0.        ]
#[-0.03141076  0.         -0.99950656]

p0 = JointTrajectoryPoint()
p0.positions = [190/180*math.pi,-60/180*math.pi,110/180*math.pi,40/180*math.pi,90/180*math.pi,-70/180*math.pi]
p0.velocities= [0.0, 0.0, 0.0, 0.0, 0.0, 0.0] 
p0.time_from_start = rospy.Duration(15)

# Append goal trajectory with p0
g.trajectory.points.append(p0)

client.send_goal(g)
print "sent the goal"
print "waiting to get there"
client.wait_for_result()
print "got there"

#just above table position is 0.6, 0.28, 0.1 (these are not the joint angles given)
