#
#   Connects REQ socket to tcp://localhost:5555
#

# Change 

import zmq

context = zmq.Context()

# create socket
# server laptop ip addr
SERVER_ADDR = ""
print("Connecting to communication server…")
socket = context.socket(zmq.REQ)
socket.connect(SERVER_ADDR)

socket.send(b"Done")

#  wait on reply
message = socket.recv()
print("Received reply [ %s ]" %  message)
# release gripper
