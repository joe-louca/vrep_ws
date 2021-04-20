import socket
import rospy
import math

class CopControl:
    def __init__(self):
        self.IP_of_robot = “192.168.0.9″
        self.PORT_of_robot = 30002          # Same port used by the server
        self.connection_state = True #False (True for testing)
        self.velocity = rospy.get_param('velocity')
        self.acceleration = 1.3962634015954636 
        self.commandsAngleList=[]
        self.connect_to_UR10()

        print('Press Ctrl-C to exit...')
        try:
            while True:
                self.get_cmd()
                self.move_cmd()
                
        except KeyboardInterrupt:
            pass

        self.disconnect_from_UR10()


    def connect_to_UR10(self):
        # Check if already connected to the robot
        if self.connection_state:
            print("Already connected to the robot on IP: " + self.IP_of_robot)
            return

        # If the program made it to here, then there is no connection yet
        print("Connecting to robot at ip: " + self.IP_of_robot)
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.connect((self.IP_of_robot, self.PORT_of_robot))
            client.wait_for_server()
            self.connection_state = True
            print("Connection established successfully")
        except:
            print("Error, could not connect at the specified IP")
            return            

    def disconnect_from_UR10(self):
        # Check if there is an active connection
        if self.connection_state == False:
            print("Already offline")
            return

        # If made it to here, then there is an active connection
        print("Disconnecting from robot")

        # Try to disconnect        
        try:
            self.sock.close()
            self.connection_state = False
            print("Disconnected successfully")
        except:
            print("Error could not disconnect")
            return

    def get_cmd(self):
        joint_cmd_rads = rospy.get_param('joint_angles')
        self.commandsAngleList.append(joint_cmd_rads)

    def move_cmd(self):
        # Check if there is an active connection
        if self.connection_state == False:
            print("Error, connect first")
            return
        
        # Check if there is a command to act on
        if len(self.commandsAngleList) > 0:
            jPos = self.commandsAngleList[0]
            #self.sock.send("movej(" + str(jPos) + ", a=" + str(self.acceleration) + ", v=" + self.velocity + ")\n")
            self.commandsAngleList.remove(self.commandsAngleList[0])

        else:
            print("No available commands to execute")

Go = CopControl()
