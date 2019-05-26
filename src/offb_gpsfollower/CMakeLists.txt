cmake_minimum_required(VERSION 2.8.3)
project(offb_gpsfollower)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  std_msgs
  mavros_msgs
  sensor_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES offb_velocity
  CATKIN_DEPENDS geometry_msgs roscpp std_msgs mavros_msgs sensor_msgs
#  DEPENDS system_lib
)


include_directories(
  ${catkin_INCLUDE_DIRS}
  INCLUDE_DIRECTORIES include
)

add_executable(${PROJECT_NAME}_node 
  src/offb_gpsfollower.cpp
  src/drone_offboard.cpp
)



## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_node
   ${catkin_LIBRARIES}
)
