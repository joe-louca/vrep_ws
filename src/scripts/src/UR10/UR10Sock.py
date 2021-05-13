#!/usr/bin/env python

# Echo client program
import socket
import time
from math import pi

HOST = '192.168.1.189' # IP entered on ur5 using the pendent
PORT = 30002 # The same port as used by the server


print("Starting Program")
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((HOST, PORT))

#st = "set_digital_out(2,False)" +"\n" #THIS WORKS

j1 = str(220.0/180*pi)
j2 = str(-65.0/180*pi)
j3 = str(50.0/180*pi)
j4 = str(0.0/180*pi)
j5 = str(75.0/180*pi)
j6 = str(0.0/180*pi)
st = "movej(["+j1+","+j2+","+j3+","+j4+","+j5+","+j6+"], a=1.39, v=1.04)" + "\n"

by = st.encode()
s.send(by)
data = s.recv(1024)

#time.sleep(0.05)

#byt = st.encode()
#s.send (byt)
#time.sleep(2)
#
#time.sleep(1)
#data = s.recv(1024)

s.close()
print("Received", repr(data))
print("Program finish")
