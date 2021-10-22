cmake_minimum_required(VERSION 2.8.3)
project(third_arm)


find_package(catkin REQUIRED COMPONENTS
rospy roscpp std_msgs message_generation dynamixel_workbench_controllers
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  )

generate_messages(
  DEPENDENCIES
  std_msgs
)


catkin_install_python(PROGRAMS scripts/WRTA_ROS_interface.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
catkin_install_python(PROGRAMS scripts/motor_config.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
catkin_install_python(PROGRAMS scripts/MAIN_CONFIG_CLASSES.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


catkin_package(
   CATKIN_DEPENDS std_msgs message_runtime dynamixel_workbench_controllers
)



