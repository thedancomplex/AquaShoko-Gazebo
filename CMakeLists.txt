cmake_minimum_required(VERSION 2.8 FATAL_ERROR)


find_package(gazebo REQUIRED)
find_package(catkin REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")
#include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
#rosbuild_init()

add_library(roboControl SHARED roboControl.cc )
target_link_libraries(roboControl ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES} ach)
