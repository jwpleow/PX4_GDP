cmake_minimum_required(VERSION 2.8.3)
project(rgbcam)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  mavros_msgs
  roscpp
  sensor_msgs
  std_msgs
)

find_package(OpenCV REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES rgbcam
#  CATKIN_DEPENDS cv_bridge image_transport mavros_msgs roscpp sensor_msgs std_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/../cv_bridge
)

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../cv_bridge cv_bridge)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


add_executable(image_converter src/image_converter.cpp)
target_link_libraries(image_converter ${catkin_LIBRARIES} ${OpenCV_LIBS} cv_bridge)
