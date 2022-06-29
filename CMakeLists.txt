cmake_minimum_required(VERSION 2.8.3)
project(third_arm)


find_package(catkin REQUIRED COMPONENTS
rospy roscpp std_msgs message_generation
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  )

#generate_messages(
#  DEPENDENCIES
#  std_msgs
#)



catkin_package(
   CATKIN_DEPENDS std_msgs pypot message_runtime
)



