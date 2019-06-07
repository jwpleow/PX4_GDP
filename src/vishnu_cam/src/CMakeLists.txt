cmake_minimum_required(VERSION 3.5)
project(tracker_mono)

find_package(OpenCV 4 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

set(CMAKE_CXX_STANDARD 14)

add_executable(aruco_tracker main.cpp tracker-ar/TrackerAR.cpp tracker-ar/TrackerAR.h tracker-base/Tracker.cpp tracker-base/Tracker.h camera-calibration/CVCalibration.cpp camera-calibration/CVCalibration.h)
target_link_libraries(aruco_tracker ${OpenCV_LIBS})

add_executable(aruco_board_tracker main.2.cpp tracker-arb/TrackerARB.cpp tracker-arb/TrackerARB.h tracker-base/Tracker.cpp tracker-base/Tracker.h camera-calibration/CVCalibration.cpp camera-calibration/CVCalibration.h)
target_link_libraries(aruco_board_tracker ${OpenCV_LIBS})

add_executable(main_ros main_ros.cpp tracker-ar/TrackerAR.cpp tracker-ar/TrackerAR.h tracker-base/Tracker.cpp tracker-base/Tracker.h camera-calibration/CVCalibration.cpp camera-calibration/CVCalibration.h)
target_link_libraries(main_ros ${OpenCV_LIBS})

add_executable(main_gazebo main.2.cpp tracker-arb/TrackerARB.cpp tracker-arb/TrackerARB.h tracker-base/Tracker.cpp tracker-base/Tracker.h camera-calibration/CVCalibration.cpp camera-calibration/CVCalibration.h)
target_link_libraries(aruco_board_tracker ${OpenCV_LIBS})

add_executable(create_board tracker-arb/create_board.cpp)
target_link_libraries(create_board  ${OpenCV_LIBS})