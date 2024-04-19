#
#   Binds REP socket to tcp://*:5555
#

import time
import zmq

def setup_server(addr="tcp://*:5555"):
    context = zmq.Context()
    socket = context.socket(zmq.REP)
    socket.bind(addr)

    while True:
        #  wait for client request
        message = socket.recv()
        print("Received request: %s" % message)

        #  drive to other robot, grip lego
        time.sleep(5)

        #  send reply back to client
        socket.send(b"Release")

setup_server()
