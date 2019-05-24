cmake_minimum_required(VERSION 2.8.3)
project(moving_gps_target)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  mavros_msgs
  roscpp
  sensor_msgs
  std_msgs
)

find_package(gazebo REQUIRED)


catkin_package(
#  CATKIN_DEPENDS geometry_msgs mavros_msgs roscpp sensor_msgs std_msgs
)



include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")
add_executable(moving_gps_target src/moving_gps_target.cc)
#add_library(moving_gps_target SHARED moving_gps_target.cc)
target_link_libraries(moving_gps_target ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} pthread)

