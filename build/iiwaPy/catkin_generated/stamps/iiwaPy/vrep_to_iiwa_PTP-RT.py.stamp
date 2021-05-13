# -*- coding: utf-8 -*-
"""
Created on Mon Mar 26 17:03:26 2018
@author: Mohammad SAFEEA
Test script of iiwaPy class.
"""
from iiwaPy import iiwaPy
import math
import time
from datetime import datetime


# returns the elapsed seconds since the start of the program

class CopControl:
    def __init__(self):
        self.start_time = datetime.now()
        self.IP_of_robot = '172.31.1.148'        
        self.connection_state = False #(True for testing)
        
        self.velocity = rospy.get_param('velocity')
        self.time_step = 0.002          # TO DO - GET PARAM
        self.commandsAngleList=[]
        self.connect_to_iiwa()

        # Move to an initial position    
        initPos=[0,0,0,-math.pi/2,0,math.pi/2,0]; # SET THIS
        initVel=[0.1]
        self.iiwa.movePTPJointSpace(initPos,initVel)

        print('Press Ctrl-C to exit...')
        if self.connection_state:
            try:
                self.t0=getSecs()                               # Start time
                self.iiwa.realTime_startDirectServoJoints()     # Start servo

                while True:
                    self.get_cmd()
                    self.move_cmd()
                    
            except KeyboardInterrupt:                           # Ctrl-C to exit
                pass
            
        self.iiwa.realTime_stopDirectServoJoints()              # Stop servo
        self.disconnect_from_iiwa()                             # Disconnect
        self.iiwa.close
        self.dt = getSecs()-t0;                                 # Total control time



    def connect_to_iiwa(self):
        # Check if already connected to the robot
        if self.connection_state:
            print("Already connected to the robot on IP: " + self.IP_of_robot)
            return

        # If the program made it to here, then there is no connection yet
        print("Connecting to robot at ip: " + self.IP_of_robot)
        try:
            self.iiwa = iiwaPy(self.IP_of_robot)
            self.connection_state = True
            print("Connection established successfully")
            
        except:
            print("Error, could not connect at the specified IP")
            return            

    def disconnect_from_iiwa(self):
        # Check if there is an active connection
        print("Disconnecting from robot")
        if self.connection_state == False:
            print("Already offline")
            return

        # If made it to here, then there is an active connection
        # Try to disconnect        
        try:
            self.iiwa.close()
            self.connection_state = False
            print("Disconnected successfully")
            
        except:
            print("Error could not disconnect")
            return

    def get_cmd(self):
        joint_cmd_rads = rospy.get_param('joint_angles')
        # TO DO - match to BRL ref frames...
        self.commandsAngleList.append(joint_cmd_rads)

    def move_cmd(self):
        t_0 = getSecs()                             # Refreshable start time
        if len(self.commandsAngleList) > 0:         # If there are commands to act on
            jPos = self.commandsAngleList[0]        # Get new joint positions
            if (getSecs()-t_0)>self.time_step:      # If elapsed time for this step > desired time_step
                self.iiwa.sendJointsPositions(jPos) # Send command

    def getSecs():
        # Gets ms since starting module
        dt = datetime.now() - self.start_time
        ms = (dt.days * 24 * 60 * 60 + dt.seconds)  + dt.microseconds / 1000000.0
        return ms
   






