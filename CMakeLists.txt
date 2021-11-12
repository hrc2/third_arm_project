cmake_minimum_required(VERSION 2.8.3)
project(third_arm)


find_package(catkin REQUIRED COMPONENTS
rospy roscpp std_msgs message_generation
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  )

generate_messages(
  DEPENDENCIES
  std_msgs
)


catkin_install_python(PROGRAMS scripts/ROS_to_third_arm.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
catkin_install_python(PROGRAMS scripts/PyPotMotorConfig.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
catkin_install_python(PROGRAMS scripts/PyPotController.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
catkin_install_python(PROGRAMS scripts/motion_planner_controller.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
catkin_install_python(PROGRAMS scripts/motion_planner_config.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
catkin_install_python(PROGRAMS scripts/inverse_kinematics.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)



catkin_package(
   CATKIN_DEPENDS std_msgs pypot message_runtime
)



