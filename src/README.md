offb is a simple offboard takeoff node       - rosrun offb offb_node

offb_velocity does takeoff and velocity commands in body coords and ENU coords    - rosrun offb_velocity node

follower is [gerald's target GPS package](https://github.com/gdlow/pixhawk-android-gps-follower)        -         roslaunch follower gazebonodes2.launch for the gazebo nodes required

three_d_cam is jiawei's 3D depth cam transformer (3d cam scrapped)          -       rosrun three_d_cam three_d_cam

lidar_converter converts the mavros/altitude.bottomclearance lidar data in gazebo from the sf10a model to /teraranger_one topic and sensor_msgs/Range message 

drone_lib is the [big drone control library](https://github.com/sirferrer/drone_lib)           -       rosrun drone_lib <mission node>


rgbcam_gazebo is [vishnu's camera package](https://github.com/vishthemenon/tracker-monorepo) that subscribes to gazebo's rgb cam data (over ROS) and converts it into OpenCV using cv_bridge for vishnu's code to work  -  rosrun rgbcam_gazebo vishnu_cam    -    NOTE: must place CalibParams640 (rename to CalibParams) & detectorparams from rgbcam_gazebo/src/data into catkin_ws/devel/lib/rgbcam_gazebo, and do the rosrun from inside catkin_ws/devel/lib/rgbcam_gazebo for the files to be picked up during execution. Must also match CalibParams with fpv_cam's model.sdf specifications!! - look inside this repo's devel/lib/rgbcam_gazebo/CalibParams.txt for the correct values to match with fpv_cam

data_writer is a node that writes rostopic data to textfiles - edit the textfile write location and build before using!  -  rosrun data_writer data_writer
