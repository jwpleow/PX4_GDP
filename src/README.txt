imu_data is a package to read mavros data

offb is a simple offboard takeoff node       - rosrun offb offb_node

offb_velocity does takeoff and velocity commands in body coords and ENU coords    - rosrun offb_velocity node

follower is gerald's target GPS package

three_d_cam is jiawei's 3D depth cam transformer

lidar_converter converts the mavros/altitude.bottomclearance lidar data in gazebo from the sf10a model to /teraranger_one topic and sensor_msgs/Range message