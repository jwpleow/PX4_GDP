cmake_minimum_required(VERSION 3.13)
project(tracker_mono)

find_package(OpenCV 4 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

set(CMAKE_CXX_STANDARD 14)

list(APPEND SRC_BASE tracker-base/Tracker.cpp camera-calibration/CVCalibration.cpp)
list(APPEND SRC_V1 tracker-ar/TrackerAR.cpp ${SRC_BASE})
list(APPEND SRC_V1_1 tracker-arb/TrackerARB.cpp ${SRC_BASE})

# Tracker - v1.0

add_executable(tracker_v1 main.cpp ${SRC_V1})
target_link_libraries(tracker_v1 ${OpenCV_LIBS})

# Tracker - v1.1

add_executable(tracker_v2 main.2.cpp ${SRC_V1_1})
target_link_libraries(tracker_v2 ${OpenCV_LIBS})

add_executable(tracker_ros main.ros.cpp ${SRC_V1_1})
target_link_libraries(tracker_ros ${OpenCV_LIBS})

add_executable(tracker_gazebo main.gazebo.cpp ${SRC_V1_1})
target_link_libraries(tracker_gazebo ${OpenCV_LIBS})