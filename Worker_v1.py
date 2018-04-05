## Spring 2018 Capstone - Autonomous Multirotor UAV
## Blake Bremer, James Murphy, Nick Bush, Matt Drury, Brad Dahlman

##      --------Revision Log-----------
# 4/4/18 21:04 Intitial Creation
# 
# 
# 
# 

''
About this code:
This is the script that will be running on the working/swarming UAVs,
all controlled by the Queen. This code will have obstacle avoidance,
but will not have any means to navigate waypoints on its own. The
workers are purely intended to maintian a specific position in
relation to the queen, or, in relation to a 'nominal point'
that the queen and all workers will have a specific relative position.

For future work on this project, a more complex algorithm that allows
for the workers to navigate on their own should the queen be lost
may be implemented.
'''

import socket
import dronekit







import socket
 
UDP_IP = "127.0.0.1"
UDP_PORT = 5005

sock = socket.socket(socket.AF_INET, # Internet
                     socket.SOCK_DGRAM) # UDP
sock.bind((UDP_IP, UDP_PORT))

while True:
    data, addr = sock.recvfrom(1024) # buffer size is 1024 bytes
    print "received message:", data
