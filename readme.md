# How to launch Gazebo with a PX4 iris model (quadcopter) connected to MAVROS

FOR UBUNTU 18.04 ONLY - Make sure Gazebo9 and ROS-melodic are installed - [Here - Gazebo with ROS Melodic](https://dev.px4.io/en/setup/dev_env_linux.html).
Also install OpenCV 4 [here](https://www.pyimagesearch.com/2018/08/15/how-to-install-opencv-4-on-ubuntu/) for vision packages: rgbcam_gazebo and vishnu_cam - delete these folders in catkin_ws/src if not wanted. (Warning, building OpenCV requires slightly over 10GB of free space! - but you can delete the built files after)

Clone this into the root directory
```
cd 
git clone https://github.com/gdp-drone/catkin_ws.git
```
Build the ROS packages using:
```
cd ~/catkin_ws
catkin clean
catkin build
```

The Gazebo simulation environment with the drone can then be launched using:
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

This builds the iris model (found in catkin_ws/Firmware/Tools/sitl_gazebo/models/iris) with a PX4 controller - look inside Firmware/launch/mavros_posix_sitl.launch for more details


A simple node to control the drone can be found in catkin_ws/src/offb_velocity - this can be ran in a new terminal:
```
source ~/catkin_ws/devel/setup.bash
rosrun offb_velocity node
```

To launch processing nodes (such as target gps conversion and depth camera transformation)
```
roslaunch follower gazebonodes2.launch 
```


Sensors added to the iris drone [look at the bottom of iris.sdf to see the additions]:
1. Downward facing LIDAR (sf10a) (publishing to /mavros/altitude -> bottom_clearance, type: mavros_msgs/Altitude, but a node converts it to type: sensor_msgs/Range and topic: /teraranger_one)
2. Forward facing depth camera (depth_camera) (publishing to /camera/depth/points, type: sensor_msgs/PointCloud2, using the kinect OpenNI plugin)
3. Downward facing FPV camera (fpv_cam) (publishing to /iris/usb_cam/image_raw, type: sensor_msgs/Image, using libgazebo_camera plugin)
4. Forward facing 3* lidars (teraranger) (publishing to /laser/scan, type: sensor_msgs/LaserScan, using libgazebo_ros_gpu_laser plugin)


An rviz configuration file is included in the repository to view the live sensor data (sensors.rviz)

To fix fixed frame error in rviz, type into a command line:
```
rosrun tf static_transform_publisher 0.0 0.0 0.0 0.0 0.0 0.0 map my_frame 100
```







## To collect data from Gazebo:
Click on the top right plot icon in Gazebo > Pull e.g. model pose data into the graphs > Run Sim > Export (Note Gazebo only seems to store the last 40 seconds)

If more data is needed try [logging](http://gazebosim.org/tutorials?tut=log_filtering) (top right LOG button in Gazebo) - and then once logged, extract the data by replacing *** and iris below with the log folder name and the model's pose wanted:
```
gz log -e -f ~/.gazebo/log/***/gzserver/state.log --filter iris.pose/*.pose > ~/mydata.log
```

## To build and attach models/sensors 
Navigate to catkin_ws/Firmware/Tools/sitl_gazebo/models/ and make your own model/sensor similar to the other models - a .sdf and a .config file is required. You can then attach a sensor to the iris model inside catkin_ws/Firmware/Tools/sitl_gazebo/models/iris/iris.sdf (scroll to the bottom of the file for examples of attaching sensors)

## To edit the simulation world
Navigate to catkin_ws/Firmware/Tools/sitl_gazebo/worlds/ - the one launched with the script at the top (roslaunch px4 mavros_posix_sitl.launch) is simulation.world. You can edit the world file being launched in the launchfiles in catkin_ws/Firmware/launch/, or edit simulation.world itself. TODO: Add wind box.

## To edit/add Gazebo-ROS plugins for use in sensors/models
Place built .so plugins in catkin_ws/Firmware/build/px4_sitl_default/build_gazebo/ - You can build these plugins as in the readme in catkin_ws/gazeboplugins. Some nifty plugins can be sourced from hector_gazebo or from catkin_ws/Firmware/Tools/sitl_gazebo/src/ - CMakeList examples can be found in catkin_ws/gazeboplugins

