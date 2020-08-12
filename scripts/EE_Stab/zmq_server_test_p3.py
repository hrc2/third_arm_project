#!/usr/bin/env python3

import math
import time
import numpy as np

import zmq

context = zmq.Context()
socket = context.socket(zmq.REP)
socket.bind("tcp://*:5555")

dat = np.random.rand(2,2)

while True:
    #  Wait for next request from client
    message = socket.recv()
    print("Received request: %s" % message)

    #  Do some 'work'
    time.sleep(1)

    #  Send reply back to client
    #dat_shape = np.array(dat.shape, dtype=np.int32)
    #msg =  [dat.dtype.name.encode(), dat_shape.tostring(), dat.tostring()]
    msg =  dat.tostring()
    socket.send(msg)