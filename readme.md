# How to launch gazebo with a px4 quad publishing to mavros

In terminal: (after building catkin_ws in the root directory from <https://dev.px4.io/en/ros/mavros_installation.html> and cloning [github/PX4/Firmware](https://github.com/PX4/Firmware) into catkin_ws/src/)

```
cd ~/catkin_ws/src/Firmware   
export GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:~/catkin_ws/build/gazebo_ros_pkgs/gazebo_plugins/         
make posix_sitl_default
source ~/catkin_ws/src/Firmware/Tools/setup_gazebo.bash ~/catkin_ws/src/Firmware ~/catkin_ws/src/Firmware/build/px4_sitl_default
source ~/catkin_ws/devel/setup.bash
source Tools/setup_gazebo.bash $(pwd) $(pwd)/build_posix_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/Tools/sitl_gazebo
roslaunch px4 mavros_posix_sitl.launch
```

This builds the iris model (found in catkin_ws/src/Firmware/Tools/sitl_gazebo/models/iris)


A simple node to control the drone can be found in catkin_ws/src/offb_velocity - this can be ran in a new terminal:
```
source ~/catkin_ws/devel/setup.bash
rosrun offb_velocity node
```

LIDAR (sf10a) and a FPV camera have been added to the base iris model (look in iris.sdf)
sf10a (lidar) publishes to /mavros/altitude -> bottom clearance
FPV camera data can be viewed using
```
source ~/catkin_ws/devel/setup.bash
rosrun rviz rviz
```
and clicking Add -> Image -> Topic: iris....







XML macros/xacros can be added in catkin_ws/src/Firmware/Tools/sitl_gazebo/models/rotors_description/urdf/iris_base.xacro)





