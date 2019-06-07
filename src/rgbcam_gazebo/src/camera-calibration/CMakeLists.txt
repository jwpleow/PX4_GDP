cmake_minimum_required(VERSION 3.13)
project(object_tracking)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

set(CMAKE_CXX_STANDARD 14)

add_executable(camera_calibration main.cpp CVCalibration.cpp)
target_link_libraries(camera_calibration ${OpenCV_LIBS})
