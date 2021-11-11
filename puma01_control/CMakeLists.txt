cmake_minimum_required(VERSION 3.0.2)
project(puma01_control)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  control_msgs
  control_toolbox
  controller_interface
  forward_command_controller
  hardware_interface
  realtime_tools
  roscpp
  std_msgs
  urdf
)

catkin_package(
  # CATKIN_DEPENDS
  #   control_msgs
  #   control_toolbox
  #   controller_interface
  #   forward_command_controller
  #   hardware_interface
  #   realtime_tools
  #   std_msgs
  #   urdf
  LIBRARIES
    ${PROJECT_NAME}
  INCLUDE_DIRS
    include
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(${PROJECT_NAME} 
  src/computed_torque_controller.cpp
)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(computed_torque src/computed_torque.cpp)
target_link_libraries(computed_torque ${catkin_LIBRARIES})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(FILES puma01_controllers_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

