cmake_minimum_required(VERSION 3.0.2)
project(puma01_usbcan)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

# find_package(Boost 
#   REQUIRED COMPONENTS 
#   system
#   thread
# )

catkin_package(
  CATKIN_DEPENDS 
    roscpp 
    std_msgs
  LIBRARIES
    ${PROJECT_NAME}
  INCLUDE_DIRS
    include
)

include_directories(
  include 
  ${catkin_INCLUDE_DIRS}
  # ${Boost_INCLUDE_DIRS}
)

# set libvs_can_api lib path
link_directories(lib)

add_library(${PROJECT_NAME}
  src/usbcan.cpp
)

target_link_libraries(
  ${PROJECT_NAME} 
  vs_can_api
  ${catkin_LIBRARIES}
  # ${Boost_LIBRARIES}
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)