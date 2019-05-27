#include "headers/data.h"
#include <cmath>

const double pi = 3.14159265358979;

data::data(float _rate)
{
    rate = ros::Rate(_rate);

    // Subscribe to Altitude Data
    altitude_sub = nh.subscribe<mavros_msgs::Altitude>("/mavros/altitude", 10, &data::altitude_cb, this);

    // Subscribe to Compass Data
    compass_sub = nh.subscribe<std_msgs::Float64>("/mavros/global_position/compass_hdg", 10, &data::heading_cb, this);

    // Subscribe to GPS Data
    gps_sub = nh.subscribe<sensor_msgs::NavSatFix>("/android/fix", 10, &data::gps_cb, this);

    // Subscribe to LiDar Data ///< need to change this to correct topic and message type once confirmed
    lidar_sub = nh.subscribe<sensor_msgs::LaserScan>("/mavros/laser/scan", 10, &data::lidar_cb, this);

    // Subscribe to IMU Data
    imu_sub = nh.subscribe<sensor_msgs::Imu>("/mavros/imu/data", 10, &data::imu_cb, this);

    // Subscribe to Position Data
    pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 10, &data::pose_cb, this);

    // Subscribe to Velocity Data
    velocity_sub = nh.subscribe<geometry_msgs::TwistStamped>("/mavros/local_position/velocity", 10, &data::velocity_cb, this);

    ///< Subscribe to target-drone relative xyz
    target_position_sub = nh.subscribe<geometry_msgs::PointStamped>("/gps_wrtdrone_position" , 10, &data::target_position_cb, this);
}

///< Yaw angle calculator (in degrees) based off target position relative to drone
double data::CalculateYawAngle(){

double yaw = atan2(target_position.point.y , target_position.point.x) * 180.0 / pi;
if (yaw < 0) return (360.0 + yaw);
else return yaw;

}


///< Target position subscriber
void data::target_position_cb(const geometry_msgs::PointStamped::ConstPtr &msg)
{
    target_position = *msg;
}
// Altitude subscriber callback function
void data::altitude_cb(const mavros_msgs::Altitude::ConstPtr &msg)
{
    infrared_altitude = *msg;
}

// Heading subscriber callback function
void data::heading_cb(const std_msgs::Float64::ConstPtr &msg)
{
    compass_heading = *msg;
}

// GPS subscriber callback function
void data::gps_cb(const sensor_msgs::NavSatFix::ConstPtr &msg)
{
    gps_raw = *msg;
}

// LiDar subscriber callback function
void data::lidar_cb(const sensor_msgs::LaserScan::ConstPtr &msg)
{
    lidar = *msg;
}

// IMU subscriber callback function
void data::imu_cb(const sensor_msgs::Imu::ConstPtr &msg)
{
    imu = *msg;
}

// Pose subscriber callback function
void data::pose_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    local_pose = *msg;
}

// Velocity subscriber callback function
void data::velocity_cb(const geometry_msgs::TwistStamped::ConstPtr &msg)
{
    local_velocity = *msg;
}