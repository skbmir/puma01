cmake_minimum_required(VERSION 3.0.2)
project(puma01_trajectory_executer)

# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  control_msgs
  geometry_msgs
  message_generation
  message_runtime
  roscpp
  rospy
  sensor_msgs
  tf
  trajectory_msgs
)

generate_messages(
  DEPENDENCIES
  actionlib_msgs
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
  message_runtime
  actionlib
  actionlib_msgs
  geometry_msgs
  trajectory_msgs
  roscpp
  rospy
  sensor_msgs
  tf
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS scripts/trajectory_executer.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
