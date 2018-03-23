#!/usr/bin/env python

import sys
import time
import copy
import numpy as np
from numpy import *
import rospy
import geometry_msgs.msg
from pyquaternion import Quaternion
import random
from std_msgs.msg import String, Float64

# API Documentation
# http://docs.ros.org/indigo/api/moveit_commander/html/classmoveit__commander_1_1move__group_1_1MoveGroupCommander.html

# kinematic constraints of the arm
linkLengths = [-0.08,0.045, 0.135] # [m]
linkConstraints = []

pub_motor1 = rospy.Publisher('/base_swivel_controller/command', Float64, queue_size=1)
pub_motor2 = rospy.Publisher('/vertical_tilt_controller/command', Float64, queue_size=1)
pub_motor3 = rospy.Publisher('/arm_extension_controller/command', Float64, queue_size=1)
pub_motor4 = rospy.Publisher('/wrist_controller/command', Float64, queue_size=1)
pub_motor5 = rospy.Publisher('/wrist_tilt_controller/command', Float64, queue_size=1)
pub_motor6 = rospy.Publisher('/gripper_controller/command', Float64, queue_size=1)

def stabilization_setup():
  # Initial setup
  # Intialize the node and name it
  
  rospy.init_node('ik_stab', anonymous=True)
  getIK()
  #rospy.Subscriber("EEPose", Pose, stabilization_callback)

  print "Moving to initial position..."
  init_vals = [-0.4, 1.3, 0.08, 0.1, 1.8]
  cmdJoints(init_vals)
  rospy.sleep(10) # give arm time to move to the initial position

  print "Starting stabilization loop..."

  # once everything is set up start the stabilization
  #stabilization_loop(init_vals)


def stabilization_loop(init_joint):
  # continuous loop
  initPose = rospy.wait_for_message("EEPose", Pose)
  init_T = poseToMatrix(initPose)

  while not rospy.is_shutdown():
      newPose = rospy.wait_for_message("EEPose", Pose)
      newT = poseToMatrix(newPose)

      tempT = np.subtract(newT,init_T)
      cmdT = np.subtract(init_T, tempT)

      newJoints = getJointAngles(cmdT)
      newJoints[2] = newJoints[2] - 0.33 # subtract extra extension length
      if valid_joint_vals(newJoints):
        cmdJoints(newJoints)
      else:
        print "===== Invalid Joint Values!!! ====="
        print newJoints
        continue


def valid_joint_vals(joint):
  # checks if the calculated joint values are within the joint limits
  if joint[0]>=-pi and joint[0]<= pi:
    if joint[1]>0 and joint[1] <= pi/2:
      if joint[2]>= 0.33 and joint[2] <= 0.48:
        if joint[3]>= -pi and joint[3]<= pi:
          if joint[4]>= 0 and joint[4]<= pi:
            return True
  # all conditions must be met, otherwise joint vals are invalid
  return False

def cmdJoints(joint):

  pub_motor1.publish(joint[0])
  pub_motor2.publish(joint[1])
  pub_motor3.publish(joint[2])
  pub_motor4.publish(joint[3])
  pub_motor5.publish(joint[4])


def getIK():

  numValid = 0
  numInvalid = 0
  sumTime = 0
  N = 100000

  for i in range(0,N):
    # generate a random set of valid joints
    joints = [0,0,0,0,0]
    joints[0] = 2*pi*random.random() - pi
    joints[1] = pi/2*random.random()
    joints[2] = 0.33 + 0.15*random.random()
    joints[3] = 2*pi*random.random() - pi
    joints[4] = pi*random.random()

    start_time = time.time()
    T = forwardKinematics(joints)

    joint = getJointAngles(T)

    if not valid_joint_vals(joint):
      numInvalid = numInvalid + 1
    else:
      #print joints
      elapsed_time = time.time() - start_time
      numValid = numValid +1
      sumTime = sumTime + elapsed_time

  avgTime = sumTime/numValid
  print "Valid Ik solutions: %s" % numValid
  print "invalid  ik solutions: %s" % numInvalid
  print "average computation time: %s" % avgTime


def poseToMatrix(pose):

  pos = pose.position
  quat = pose.orientation

  x = pos.x
  y = pos.y
  z = pos.z

  q = Quaternion(quat.w, quat.x, quat.y, quat.z)

  T = q.transformation_matrix

  T[0][3] = x
  T[1][3] = y
  T[2][3] = z
  return T.astype(np.float64)

def getJointAngles(T):
  linkLengths = [-0.08,0.045, 0.135]; # [m]
  t_pred = np.zeros(5)
  tVector = np.reshape(T,(1,16))
  goalVec = tVector[0]
  #rospy.loginfo(goalVec)

  t_pred[0] = calcTheta1(goalVec,linkLengths[2])

  c4, t_pred[1] = calcTheta2(goalVec, t_pred[0])

  t_pred[3] = calcTheta4(goalVec, t_pred[1], c4)

  t_pred[4] = calcTheta5(goalVec, t_pred[1], c4)

  t_pred[2] = calcTheta3(goalVec, t_pred[1], t_pred[3], t_pred[4], linkLengths)

  return t_pred

def calcTheta1(vars, link3):
  r1y = vars[4]
  r1x = vars[0]
  py = vars[7]
  px = vars[3]

  theta1 = np.arctan2(py- link3*r1y, px - link3*r1x)
  return theta1

def calcTheta2(vars, theta1):
  r2x = vars[1]
  r2y = vars[5]
  r2z = vars[9]

  A = np.array([[sin(theta1), cos(theta1)*r2z],[-cos(theta1), sin(theta1)*r2z]],dtype=np.float64)
  b = np.array([[r2x],[r2y]]) 
  #sol1, sol2 = np.linalg.lstsq(A,b)[0]
  sol = np.linalg.solve(A,b)
  sol1 = sol[0]
  sol2 = sol[1]

  # This also gives us cos(theta4)
  c4 = sol1

  # check range: [0,pi/2]
  candidate = np.arctan2(1,sol2)
  if candidate >0 and candidate <pi/2:
    theta2 = candidate
  elif candidate>pi/2 and candidate <= pi:
    theta2 = -candidate + pi
  elif candidate<-pi/2 and candidate >-pi:
    theta2 = pi+ candidate
  else:
    theta2 = -candidate
  return c4, theta2

def calcTheta3(vars, theta2, theta4, theta5, links):
  # Calculate joint variable 3 = extension length
  # range [0.33, 0.45]

  pz = vars[11]
  l1 = links[0]
  l2 = links[1]
  l3 = links[2]

  extension = (-pz + l1 - l3*cos(theta2)*sin(theta5) - l3*cos(theta4)*cos(theta5)*sin(theta2) - l2*cos(theta2))/cos(theta2)
  return extension

def calcTheta4(vars, theta2, c4):
  # range [-pi,pi]

  r2z = vars[9]
  s4 = -r2z/sin(theta2)
  theta4 = np.arctan2(s4,c4)
  return theta4

def calcTheta5(vars, theta2, c4):
  # range [0,pi]

  r1z = vars[8]
  r3z = vars[10]

  A = np.array([[-cos(theta2), -c4*sin(theta2)],[-c4*sin(theta2), cos(theta2)]])
  sol = np.linalg.solve(A,np.array([[r1z],[r3z]]))

  theta5 = np.arctan2(sol[0],sol[1])

  return theta5

def isSingular(joint):
  # checks if a set of joint parameters are singular or not
  # Returns True if joint values lead to a singular FK solution

  T = forwardKinematics(joint)

  if (np.linalg.matrix_rank(T)== 4):
    return False
  else:
    return True


def forwardKinematics(joint):
  # Performs forward kinematics on the third_arm using the Denavit-Hartenberg method

  FK = np.identity(4) # Identity matrix
  alphas = [pi/2, pi/2, 0, pi/2, pi/2, 0]
  ais = np.array([0, 0, 0, 0, 0, linkLengths[2]])
  D = [linkLengths[0], 0, joint[2], linkLengths[1], 0, 0]
  thetas = [joint[0], joint[1], pi, joint[3], joint[4], 0]
  #thetas[2] = pi # theta 3 is not a rotation

  for i in range(6):
    new_Transform = np.array([[cos(thetas[i]), -cos(alphas[i])*sin(thetas[i]), sin(alphas[i])*sin(thetas[i]), ais[i]*cos(thetas[i])],
                     [sin(thetas[i]), cos(alphas[i])*cos(thetas[i]), -sin(alphas[i])*cos(thetas[i]), ais[i]*sin(thetas[i])],
                     [0, sin(alphas[i]), cos(alphas[i]), D[i]],
                     [0, 0, 0, 1]])
    FK = np.dot(FK,new_Transform)

  return FK



if __name__=='__main__':
  try:
    stabilization_setup()
  except rospy.ROSInterruptException:
    pass
