# -*- coding: utf-8 -*-

from iiwaPy import iiwaPy
import rospy
import math

class CopControl:
    def __init__(self):
        self.IP_of_robot = '172.31.1.148'        
        self.connection_state = False #(True for testing)      
        self.commandsAngleList=[]
        self.connect_to_iiwa()

        print('Press Ctrl-C to exit...')
        if self.connection_state:
            try:
                while True:
                    self.get_cmd()
                    self.move_cmd()
                    
            except KeyboardInterrupt:
                pass

        self.disconnect_from_iiwa()


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
        joint_cmd_rads = rospy.get_param('joint_vel')
        # TO DO - match to BRL ref frames...
        self.commandsAngleList.append(joint_cmd_rads)

    def move_cmd(self):
        # Check if there is an active connection
        if self.connection_state == False:
            print("Error, connect first")
            return
        
        # Check if there is a command to act on
        if len(self.commandsAngleList) > 0:
            jVel = self.commandsAngleList[0]
            self.iiwa.moveVelJointSpace(jVel)
            self.commandsAngleList.remove(self.commandsAngleList[0])

        else:
            print("No available commands to execute")

Go = CopControl()
