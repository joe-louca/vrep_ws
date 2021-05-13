import socket
import rospy
from math import pi

class CopControl:
    def __init__(self):
        self.HOST = '192.168.1.189'
        self.PORT = 30002          # Same port used by the server
        
        self.connection_state = False##(True for testing)
        #self.velocity = rospy.get_param('velocity')
        #self.velocity = self.velocity[0]
        self.velocity = 2.0
        self.acceleration = 1.0
        self.commandsAngleList=[]
        self.connect_to_UR10()

        
        print('Press Ctrl-C to exit...')
        try:
            while True:
                # get command from coppelia
                joint_cmd_rads = rospy.get_param('joint_angles')
                joint_cmd_rads[1] -= pi/2 # offset to match coppeliaSim to BRL UR10 ref frames
                joint_cmd_rads[3] -= pi/2 # offset to match coppeliaSim to BRL UR10 ref frames

                # send command to ur10
                st = "movej(" + str(joint_cmd_rads) + ", a=" + str(self.acceleration) + ", v=" + str(self.velocity) + ")\n"
                by = st.encode()
                self.sock.send(by)
                data = self.sock.recv(1024)

                # add a wait time....
                
                #self.get_cmd()
                #self.move_cmd()
                
        except KeyboardInterrupt:
            pass

        self.disconnect_from_UR10()


    def connect_to_UR10(self):
        # Check if already connected to the robot
        if self.connection_state:
            print("Already connected to the robot on IP: " + self.HOST)
            return

        # If the program made it to here, then there is no connection yet
        print("Connecting to robot at ip: " + self.HOST)
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.connect((self.HOST, self.PORT))
            #client.wait_for_server()
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
        joint_cmd_rads[1] -= pi/2 # offset to match coppeliaSim to BRL UR10 ref frames
        joint_cmd_rads[3] -= pi/2 # offset to match coppeliaSim to BRL UR10 ref frames
        self.commandsAngleList.append(joint_cmd_rads)

    def move_cmd(self):
        # Check if there is an active connection
        if self.connection_state == False:
            print("Error, connect first")
            return
        
        # Check if there is a command to act on
        if len(self.commandsAngleList) > 0:
            jPos = self.commandsAngleList[0]
            st = "movej(" + str(jPos) + ", a=" + str(self.acceleration) + ", v=" + str(self.velocity) + ")\n"
            #print(st)
            by = st.encode()
            self.sock.send(by)
            self.commandsAngleList.remove(self.commandsAngleList[0])

        else:
            print("No available commands to execute")

Go = CopControl()
