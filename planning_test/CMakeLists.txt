cmake_minimum_required(VERSION 3.0.2)
project(planning_test)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  control_msgs
  geometry_msgs
  moveit_msgs
  moveit_visual_tools
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf2
  tf2_geometry_msgs
  tf2_msgs
  trajectory_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES planning_test
#  CATKIN_DEPENDS actionlib actionlib_msgs control_msgs geometry_msgs moveit moveit_msgs moveit_visual_tools roscpp rospy sensor_msgs std_msgs tf2 tf2_geometry_msgs tf2_msgs trajectory_msgs
#  DEPENDS system_lib
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

add_executable(addfloor src/addfloor.cpp)
target_link_libraries(addfloor ${catkin_LIBRARIES})
install(TARGETS addfloor DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
