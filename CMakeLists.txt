cmake_minimum_required(VERSION 3.2)
project(dsp_sdk)

add_definitions(-std=c++11)

find_package(catkin REQUIRED)
find_package(Eigen3 REQUIRED)

# set optimization level
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O2")

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
)

include_directories(
  include 
  ${EIGEN3_INCLUDE_DIRS}
)

# Comm main library definition
add_library(${PROJECT_NAME} SHARED
  src/DspHandler.cc
)

#set_target_properties(${PROJECT_NAME} PROPERTIES PREFIX "")

install(TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}
)
