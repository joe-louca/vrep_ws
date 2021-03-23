# Echo client program
import socket
import time

HOST = “192.168.0.9″    # The remote host
PORT = 30002          # The same port as used by the server

print “Starting Program”

count = 0
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((HOST, PORT))

while (count < 1):
	s.send (“movej(joint_space, a=1.3962634015954636, v=1.0471975511965976)” + “\n”)
	time.sleep(1/rate)


data = s.recv(1024)
s.close()
print (“Received”, repr(data))
print “Program finish”
