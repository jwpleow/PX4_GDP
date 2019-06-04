# How to launch gazebo with a px4 iris model (quadcopter) publishing to mavros

Make sure gazebo9 and ros-melodic are installed!

Clone this into the root directory
```
cd 
git clone https://github.com/gdp-drone/catkin_ws.git
```

The gazebo simulation environment with the drone can then be launched using:
```
cd ~/catkin_ws/Firmware   
export GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:~/catkin_ws/build/gazebo_ros_pkgs/gazebo_plugins/         
source ~/catkin_ws/Firmware/Tools/setup_gazebo.bash ~/catkin_ws/Firmware ~/catkin_ws/Firmware/build/px4_sitl_default
source ~/catkin_ws/devel/setup.bash
source Tools/setup_gazebo.bash $(pwd) $(pwd)/build_posix_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)
export LIBGL_ALWAYS_SOFTWARE=1
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/Tools/sitl_gazebo
roslaunch px4 mavros_posix_sitl.launch
```

This builds the iris model (found in catkin_ws/Firmware/Tools/sitl_gazebo/models/iris) with a px4 controller - look inside Firmware/launch/mavros_posix_sitl.launch for more details


A simple node to control the drone can be found in catkin_ws/src/offb_velocity - this can be ran in a new terminal:
```
source ~/catkin_ws/devel/setup.bash
rosrun offb_velocity node
```

To launch processing nodes (such as target gps conversion and depth camera transformation)
```
roslaunch follower gazebonodes.launch 
```

Sensors added to the iris drone [look at the bottom of iris.sdf to see the additions]:
1. Downward facing LIDAR (sf10a) (publishing to /mavros/altitude -> bottom_clearance, type: mavros_msgs/Altitude, but a node converts it to type: sensor_msgs/Range and topic: /teraranger_one)
2. Forward facing depth camera (depth_camera) (publishing to /camera/depth/points, type: sensor_msgs/PointCloud2, using the kinect OpenNI plugin)
3. Downward facing FPV camera (fpv_cam) (publishing to /iris/usb_cam/image_raw, type: sensor_msgs/Image, using libgazebo_camera plugin)
4. Forward facing 3* lidars (teraranger) (publishing to /laser/scan, type: sensor_msgs/LaserScan, using libgazebo_ros_gpu_laser plugin)


An rviz configuration file is included in the repository to view the live sensor data (sensors.rviz)

To fix fixed frame error in rviz:
```
rosrun tf static_transform_publisher 0.0 0.0 0.0 0.0 0.0 0.0 map my_frame 100
```


XML macros/xacros can be added in catkin_ws/src/Firmware/Tools/sitl_gazebo/models/rotors_description/urdf/iris_base.xacro


