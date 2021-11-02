cmake_minimum_required(VERSION 3.0.2)
project(puma01_gazebo)


find_package(catkin REQUIRED COMPONENTS
  controller_manager
  gazebo_ros
  gazebo_ros_control
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES puma01_gazebo
#  CATKIN_DEPENDS controller_manager gazebo_ros gazebo_ros_control
#  DEPENDS system_lib
)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})


