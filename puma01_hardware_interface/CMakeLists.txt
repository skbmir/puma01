cmake_minimum_required(VERSION 3.0.2)
project(puma01_hardware_interface)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  control_msgs
  controller_manager
  hardware_interface
  message_generation
  ros_control_boilerplate
  roscpp
  std_msgs
  geometry_msgs
  urdf
  actionlib_msgs 
  actionlib
  puma01_force
  vscan_usbcan_api
)

catkin_package(
  # CATKIN_DEPENDS
  # control_msgs
  # controller_manager
  # hardware_interface
  # message_generation
  # ros_control_boilerplate
  # roscpp
  # std_msgs
  # urdf
  LIBRARIES
    ${PROJECT_NAME}
  INCLUDE_DIRS
    include
)

include_directories(include ${catkin_INCLUDE_DIRS})

# puma01_hwi
add_executable(puma01_hwi 
  include/puma01_hardware_interface/puma01_hwi.h
  src/puma01_hwi.cpp
  src/puma01_hwi_main.cpp
)
add_dependencies(puma01_hwi ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(puma01_hwi
  ${catkin_LIBRARIES}
)

# puma01_hwi_test
add_executable(puma01_hwi_test 
  include/puma01_hardware_interface/puma01_hwi.h
  src/puma01_hwi_test.cpp
  src/puma01_hwi_main.cpp
)
add_dependencies(puma01_hwi_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(puma01_hwi_test
  ${catkin_LIBRARIES}
)

# hwi_pd_plus_fw_test
add_executable(hwi_pd_plus_fw_test 
  include/puma01_hardware_interface/puma01_hwi.h
  src/hwi_pd_plus_fw_test.cpp
  src/puma01_hwi_main.cpp
)
add_dependencies(hwi_pd_plus_fw_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(hwi_pd_plus_fw_test
  ${catkin_LIBRARIES}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
