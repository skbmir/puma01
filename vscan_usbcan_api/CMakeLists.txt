cmake_minimum_required(VERSION 3.0.2)
project(vscan_usbcan_api)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

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
)

# MAYBE NEED TO TRY DIFFERENT APPROACHES FOR LINKING VS_CAM_API LIB!!!!!!!!!1
# pthread library is required for linking against libvs_can_api.a
set(CMAKE_THREAD_PREFER_PTHREAD TRUE)
set(THREADS_PREFER_PTHREAD_FLAG TRUE)

find_package(Threads REQUIRED)
find_library(
	VS_CAN_API
	libvs_can_api.so
	HINTS lib
	DOC "vs_can_api")
message(STATUS "VSCAN API Library found: ${VS_CAN_API}")


# set libvs_can_api lib path
# link_directories(lib)

add_library(${PROJECT_NAME}
  src/usbcan.cpp
)

# rough way to set fPIC -flags
# set_target_properties(vs_can_api PROPERTIES COMPILE_FLAGS "-fPIC")
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")

# set(CMAKE_POSITION_INDEPENDENT_CODE ON)
# set_property(TARGET vs_can_api PROPERTY POSITION_INDEPENDENT_CODE ON)


# target_link_libraries(
#   ${PROJECT_NAME} 
#   vs_can_api
#   ${catkin_LIBRARIES}
# )

target_link_libraries (${PROJECT_NAME} 
  ${VS_CAN_API} 
  ${catkin_LIBRARIES} 
  Threads::Threads )

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)