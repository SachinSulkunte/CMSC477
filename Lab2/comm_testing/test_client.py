#
#   Hello World client in Python
#   Connects REQ socket to tcp://localhost:5555
#   Sends "Hello" to server, expects "World" back
#

import zmq

context = zmq.Context()

#  create socket
print("Connecting to communication server…")
socket = context.socket(zmq.REQ)
socket.connect("tcp://localhost:5555")

socket.send(b"Done")

#  get reply
message = socket.recv()
print("Received reply [ %s ]" %  message)
# release gripper
