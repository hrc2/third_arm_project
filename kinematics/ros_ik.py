# Function that takes in a universal transform matrix in the base frame and
# returns joint angles for the HRC2 third_arm
# Math by Vighnesh Vatsal
# Implementation by Kevin Kruempelstaedter

import math
from sympy import *

linkLengths = [-0.08,0.045, 0.135]; # [m]
linkConstraints = [];


test1 = [0 0 0 0; 0 0 0 0; 0 0 0 0; 0 0 0 0]

stuff = calcTheta1(test1,lnikLengths[2])

def getJointAngles(T):
	predictedAngles[1] = 




def calcTheta1(vars, linkLength):
	r1y = vars[1]
	r1x = vars[0]
	py = vars[13]
	px = vars[12]

	theta1 = math.atan2(py- linkLength*r1y, px - linkLength*r1x)
	return theta1

def reCalcTheta1(vars, theta2, c4):
	r2x = vars[4]
	r2y = vars[5]
	r2z = vars[6]

	A = [c4, cot(theta2)*r2z; cot(theta2)*r2z, -c4]

	avec = numpy.linalg.solve(A,[r2x;r2y])

	theta1 = atan2(avec[0],avec[1])
	return theta1
