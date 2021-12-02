cmake_minimum_required(VERSION 3.0.2)
project(usbcan_test)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  puma01_usbcan
  roscpp
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS 
    puma01_usbcan
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(usbcan_test 
  src/usbcan_test.cpp
)
add_dependencies(usbcan_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(usbcan_test
  ${catkin_LIBRARIES}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)