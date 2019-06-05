cmake_minimum_required(VERSION 2.8.3)
project(rgbcam_gazebo)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  sensor_msgs
  std_msgs
  )
find_package(OpenCV 4 REQUIRED)


###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
catkin_package()
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)
## Declare a cpp library
# add_library(ahb_ros_opencv_dynamic_reconfigure_example
#   src/${PROJECT_NAME}/ahb_ros_opencv_dynamic_reconfigure_example.cpp
# )

## Declare a cpp executable
add_executable(image_converter src/image_converter.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(ahb_ros_opencv_dynamic_reconfigure_example_node ahb_ros_opencv_dynamic_reconfigure_example_generate_messages_cpp)

# make sure configure headers are built before any node using them
#add_dependencies(ahb_ros_opencv_dynamic_reconfigure_example ${PROJECT_NAME}_gencfg)


## Specify libraries to link a library or executable target against
target_link_libraries(image_converter
  ${catkin_LIBRARIES} ${OpenCV_LIBRARIES}
)
include_directories(${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})