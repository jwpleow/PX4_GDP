#ifndef DATA_H
#define DATA_H

#include <ros/ros.h>
#include <mavros_msgs/Altitude.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/LaserScan.h>
#include <boost/circular_buffer.hpp>
#include <rosbag/bag.h>


class data
{
public:
    // Data
    mavros_msgs::Altitude altitude;
    std_msgs::Float64 compass_heading;
    sensor_msgs::NavSatFix gps_raw;
    sensor_msgs::LaserScan lidar;
    sensor_msgs::Imu imu;
    geometry_msgs::PoseStamped local_pose;
    geometry_msgs::TwistStamped local_velocity;
    geometry_msgs::TwistStamped target_abs_velocity;        ///< target velocity (ENU)
    geometry_msgs::PointStamped target_position_relative;   ///< target position relative to drone (ENU)
    sensor_msgs::NavSatFix target_gps;                      ///< target gps (LLA)
    geometry_msgs::Twist vishnu_cam_data;                   ///< Vishnu's cam data that says the ARtag position in body frame
    std_msgs::Bool vishnu_cam_detection;                    ///< Vishnu's cam boolean which tells if the ARtag is detected


    float CalculateYawAngleToTarget();              ///< calculates yaw angle for drone to face the target


    ros::Rate GetRate() { return rate; }                    ///< added to get the rate

    // Constructors
    data(){};
    data(float _rate);
    void start_rosbag();
private:
    // Hidden methods
    ros::NodeHandle nh;
    ros::Rate rate = ros::Rate(25.0);
    rosbag::Bag bag;
    bool save_data;

    ros::Subscriber compass_sub;
    ros::Subscriber altitude_sub;
    ros::Subscriber gps_sub;
    ros::Subscriber imu_sub;
    ros::Subscriber pose_sub;
    ros::Subscriber velocity_sub;
    ros::Subscriber lidar_sub;
    ros::Subscriber target_position_relative_sub;       ///< target position relative to drone
    ros::Subscriber target_abs_velocity_sub;            ///< target velocity 
    ros::Subscriber target_gps_sub;                     ///< target gps
    ros::Subscriber vishnu_cam_data_sub;                ///< vishnu cam data
    ros::Subscriber vishnu_cam_detection_sub;           ///< vishnu cam detection boolean


    void vishnu_cam_data_cb(const geometry_msgs::Twist::ConstPtr &msg);                     ///< Callback for vishnu cam data
    void vishnu_cam_detection_cb(const std_msgs::Bool::ConstPtr &msg);                      ///< Callback for vishnu cam detection
    void target_gps_cb(const sensor_msgs::NavSatFix::ConstPtr& msg);                        ///< Callback for target gps
    void target_position_relative_cb(const geometry_msgs::PointStamped::ConstPtr& msg);     ///< Callback for target-drone relative xyz
    void target_abs_velocity_cb(const geometry_msgs::TwistStamped::ConstPtr& msg);          ///< Callback for target xyz from drone origin
    void altitude_cb(const mavros_msgs::Altitude::ConstPtr& msg);
    void heading_cb(const std_msgs::Float64::ConstPtr& msg); 
    void gps_cb(const sensor_msgs::NavSatFix::ConstPtr& msg);
    void imu_cb(const sensor_msgs::Imu::ConstPtr& msg);
    void pose_cb(const geometry_msgs::PoseStamped::ConstPtr &msg);
    void velocity_cb(const geometry_msgs::TwistStamped::ConstPtr &msg);
    void lidar_cb(const sensor_msgs::LaserScan::ConstPtr &msg);
    std::string get_log_name();
};

#endif /* DATA_H */
