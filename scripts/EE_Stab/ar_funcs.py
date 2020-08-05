#!/usr/bin/env python
import rospy

import math
import time
import numpy as np

ARFILE = '/home/hriclass/catkin_ws/src/third_arm/scripts/EE_Stab/AR_params_wiping.csv'

def ar_forecast(data, k):
	ar_params = np.genfromtxt(ARFILE, delimiter=',')
	A = -ar_params[3:, 1:-1]
	eps = ar_params[3:, -1]

	base = data[:, 0:3]
	elbow = data[:, 6:9]
	wrist = data[:, 9:12]

	base_pred = np.zeros([k, 3])
	elbow_pred = np.zeros([k, 3])
	wrist_pred = np.zeros([k, 3])
	
	for j in range(3):
		base_pred[:,j] = ar_compute(np.ravel(base[:,j]), np.ravel(A[j,:]), eps[j], k)
		elbow_pred[:,j] = ar_compute(np.ravel(elbow[:,j]), np.ravel(A[j,:]), eps[j], k)
		wrist_pred[:,j] = ar_compute(np.ravel(wrist[:,j]), np.ravel(A[2+j,:]), eps[2+j], k)

	return base_pred, elbow_pred, wrist_pred

def ar_compute(data, A, eps, k):
	outvec = np.zeros([k,1])
	query = np.flipud(data)
	
	for i in range(k):		
		result = np.dot(A, query) + np.random.normal(0, np.sqrt(eps))
		outvec[i,0] = result
		r = np.roll(query, 1)
		r[0] = result
		query = r

	return np.ravel(outvec)





