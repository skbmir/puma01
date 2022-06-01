cmake_minimum_required(VERSION 3.0.2)
project(usbcan_test)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  vscan_usbcan_api
  roscpp
  std_msgs
  sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS 
    vscan_usbcan_api
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

add_executable(usbcan_test_write 
  src/usbcan_test_write.cpp
)
add_dependencies(usbcan_test_write ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(usbcan_test_write
  ${catkin_LIBRARIES}
)

add_executable(three_dof_test 
  src/three_dof_test.cpp
)
add_dependencies(three_dof_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(three_dof_test
  ${catkin_LIBRARIES}
)

add_executable(send_err_test 
  src/send_err_test.cpp
)
add_dependencies(send_err_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(send_err_test
  ${catkin_LIBRARIES}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)