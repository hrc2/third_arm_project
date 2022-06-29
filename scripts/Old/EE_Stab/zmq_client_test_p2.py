#!/usr/bin/env python
import rospy

import math
import time
import numpy as np

import zmq

context = zmq.Context()

#  Socket to talk to server
print("Connecting to hello world server")
socket = context.socket(zmq.REQ)
socket.connect("tcp://localhost:5555")

#  Do 10 requests, waiting each time for a response
for request in range(10):
    print("Sending request %s " % request)
    socket.send(b"Hello")

    #  Get the reply.
    msg = socket.recv()
    #dtype_name = msg[0].decode()
    #shape = np.fromstring(msg[1], np.int32)
    array = np.fromstring(msg, dtype=float).reshape(2, 2)
    #buf = memoryview(message)    
    
    print("Received array : ", array)