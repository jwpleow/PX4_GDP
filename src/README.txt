imu_data is a package to read mavros data

offb is a simple offboard takeoff node       - rosrun offb offb_node

offb_velocity does takeoff and velocity commands in body coords and ENU coords    - rosrun offb_velocity node

follower is gerald's target GPS package

three_d_cam is jiawei's 3D depth cam transformer (3d cam scrapped)

lidar_converter converts the mavros/altitude.bottomclearance lidar data in gazebo from the sf10a model to /teraranger_one topic and sensor_msgs/Range message (NOT NEEDED!! sigh)

drone_lib is the big drone control library

vishnu_cam is the vision algorthm - pulls camera data from a camera connected via USB and publishes the processed data (position of the ARtag wrt to the drone in body frame) rosrun vishnu_cam vishnu_cam

rgbcam_gazebo is the package that subscribes to gazebo's rgb cam data (over ROS) and converts it into OpenCV using cv_bridge for vishnu's code to work

data_writer is a node (rosrun data_writer data_writer) that writes to textfiles - edit the textfile write location and build again before using!!