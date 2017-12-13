#!/usr/bin/env python

import sys
import time
import copy
import numpy as np
from numpy import *
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from pyquaternion import Quaternion
from random import randint
from std_msgs.msg import String

# API Documentation
# http://docs.ros.org/indigo/api/moveit_commander/html/classmoveit__commander_1_1move__group_1_1MoveGroupCommander.html

# kinematic constraints of the arm
linkLengths = [-0.08,0.045, 0.135] # [m]
linkConstraints = []

def third_arm_move_group():
  ## BEGIN_TUTORIAL
  ##
  ## Setup
  ## ^^^^^
  ## CALL_SUB_TUTORIAL imports
  ##
  ## First initialize moveit_commander and rospy.
  print "============ Starting tutorial setup"
  moveit_commander.roscpp_initialize(sys.argv)
  rospy.init_node('third_arm_move_group',
                  anonymous=True)

  ## Instantiate a RobotCommander object.  This object is an interface to
  ## the robot as a whole.
  robot = moveit_commander.RobotCommander()

  ## Instantiate a PlanningSceneInterface object.  This object is an interface
  ## to the world surrounding the robot.
  scene = moveit_commander.PlanningSceneInterface()

  ## Instantiate a MoveGroupCommander object.  This object is an interface
  ## to one group of joints.  In this case the group is the joints in the left
  ## arm.  This interface can be used to plan and execute motions on the left
  ## arm.
  group = moveit_commander.MoveGroupCommander("full_arm")


  ## We create this DisplayTrajectory publisher which is used below to publish
  ## trajectories for RVIZ to visualize.
  display_trajectory_publisher = rospy.Publisher(
                                      '/move_group/display_planned_path',
                                      moveit_msgs.msg.DisplayTrajectory,
                                      queue_size=20)

  ## Wait for RVIZ to initialize. This sleep is ONLY to allow Rviz to come up.
  print "============ Waiting for RVIZ..."
  #rospy.sleep(2)
  print "============ Starting tutorial "

  ## Getting Basic Information
  ## ^^^^^^^^^^^^^^^^^^^^^^^^^
  ##
  ## We can get the name of the reference frame for this robot
  print "============ Reference frame: %s" % group.get_planning_frame()

  ## We can also print the name of the end-effector link for this group
  print "============ End effector: %s" % group.get_end_effector_link()

  ## We can get a list of all the groups in the robot
  #print "============ Robot Groups:"
  #print robot.get_group_names()
  print "============ Initial Robot Pose:"
  pose = group.get_current_pose()
  print pose

  ## Sometimes for debugging it is useful to print the entire state of the
  ## robot.
  print "======= Moving robot to start state ======="
  group_variable_values = group.get_current_joint_values()
  group_variable_values[0] = -0.4
  group_variable_values[1] = 1.3
  group_variable_values[2] = 0.1
  group_variable_values[3] = 0.1
  group_variable_values[4] = 1.8
  print "============ Moving to"
  group.set_joint_value_target(group_variable_values)
  print group_variable_values
  group.go()
  rospy.sleep(5)
  group.stop()

  print "============ Printing robot state"
  print robot.get_current_state()


  print "============Current EE pose"
  print group.get_pose_reference_frame()

  pose = group.get_current_pose()
  oldPosition = pose.pose.position
  print "XYZ Position"
  print oldPosition
  print "orientation"
  print pose.pose.orientation
  oldPose = pose.pose

  print "====== move from pose position ======"
  group_variable_values = group.get_current_joint_values()
  group_variable_values[0] = 0.4
  group_variable_values[1] = 0.4
  group_variable_values[2] = .11
  group_variable_values[3] = 0.4
  group_variable_values[4] = 0.4
  print "============ Moving to"
  group.set_joint_value_target(group_variable_values)
  print group_variable_values
  group.go()
  rospy.sleep(5)
  group.stop()
  print "====== Use IK to get new joint vals ====="
  print "Moving to previous position using position coordinates only"

  ik_joints = getIK(oldPose)

  print "==================Joint Angles==================="
  print ik_joints
  group.set_joint_value_target(ik_joints)

  group.go()
                               
  print "============ Waiting while RVIZ displays plan2..."
  rospy.sleep(5)
  group.stop()
  ## When finished shut down moveit_commander.
  moveit_commander.roscpp_shutdown()

  ## END_TUTORIAL

  print "============ STOPPING"



  while True:
    # stabilization loop
    


def getIK(data):
  start_time = time.time()
  # convert given joint values into Ik joint space
  print "=======================received: "
  print data

  T = poseToMatrix(data)

  joint = getJointAngles(T)
  #rospy.loginfo('==================Joint Angles===================')
  #rospy.loginfo(joint)

  if isSingular(joint):
    rospy.loginfo('WARNING: Calculated IK solution is singular!')
  else:
    elapsed_time = time.time() - start_time
    rospy.loginfo('IK solution calculated in %s seconds' % elapsed_time)

  FK = forwardKinematics(joint)
  rospy.loginfo('Forward Kinematics')
  #rospy.loginfo(FK)

  # convert the raw joint values into joint values in move_it joint space
  move_it_joint = [joint[0], joint[1], joint[2] - 0.33, joint[3], joint[4]]
  return move_it_joint

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
    third_arm_move_group()
  except rospy.ROSInterruptException:
    pass
