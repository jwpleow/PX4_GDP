cmake_minimum_required(VERSION 2.8.3)
project(data_writer)                                                           
find_package(catkin REQUIRED COMPONENTS
    roscpp
    sensor_msgs
    geometry_msgs
    mavros_msgs
    std_msgs
)
catkin_package()
include_directories(
    ${catkin_INCLUDE_DIRS}                                                           
)
include_directories(include ${catkin_INCLUDE_DIRS})
add_executable(data_writer src/data_writer.cpp)
target_link_libraries(data_writer ${catkin_LIBRARIES}) 