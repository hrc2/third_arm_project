#!/usr/bin/env python
import rospy

import math
import time
import numpy as np
import zmq

def rnn_forecast(data, k, sock):
	sock.send(data.tostring())
	msg = sock.recv()
	data = np.fromstring(msg, dtype=float)
	pred = data.reshape(k,9)
	return pred


if __name__ == '__main__':

	context = zmq.Context()
	sock = context.socket(zmq.REQ)
	sock.connect("tcp://localhost:5555")

	init_dat = np.random.rand(60,9)
	sock.send(init_dat.tostring())

	while True:
		msg = sock.recv()
		data = np.fromstring(msg, dtype=float)
		pred = data.reshape(10,9)
		print('Received Prediction: ', pred)
		#time.sleep(1)
		dat = np.random.rand(50,9)
		sock.send(dat.tostring())


