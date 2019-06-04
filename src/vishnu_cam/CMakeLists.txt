cmake_minimum_required(VERSION 2.8.3)
project(vishnu_cam)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  mavros_msgs
  roscpp
  std_msgs
)

find_package(OpenCV 4 REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES vishnu_cam
#  CATKIN_DEPENDS geometry_msgs mavros_msgs roscpp std_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS} 
  ${OpenCV_INCLUDE_DIRS}
)

set(CMAKE_CXX_STANDARD 14)

add_executable(vishnu_cam_node src/main_ros.cpp src/tracker-ar/TrackerAR.cpp src/tracker-ar/TrackerAR.h src/tracker-base/Tracker.cpp src/tracker-base/Tracker.h src/camera-calibration/CVCalibration.cpp src/camera-calibration/CVCalibration.h)
target_link_libraries(vishnu_cam_node ${catkin_LIBRARIES} ${OpenCV_LIBS})

