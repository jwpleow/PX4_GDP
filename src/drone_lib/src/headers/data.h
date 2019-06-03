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
#include <sensor_msgs/LaserScan.h>
#include <boost/circular_buffer.hpp>
#include <rosbag/bag.h>

///< For Depthcam
// #include <sensor_msgs/PointCloud2.h>
// #include <sensor_msgs/PointCloud.h>
// #include <sensor_msgs/point_cloud_conversion.h>
// #include <pcl_conversions/pcl_conversions.h>
// #include <pcl/point_types.h>
// #include <pcl/PCLPointCloud2.h>
// #include <pcl_ros/point_cloud.h>
// #include <pcl/conversions.h>
// #include <pcl_ros/transforms.h>


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
    geometry_msgs::PointStamped target_position;            ///< target position relative to drone origin
    geometry_msgs::PointStamped target_position_relative;   ///< target position relative to drone
    sensor_msgs::NavSatFix target_gps;                      ///< target gps


    // sensor_msgs::PointCloud2 depth_cam_pc2; ///< transformed depth cam data 
    // pcl::PointCloud<pcl::PointXYZ>::Ptr depth_cam_cloud{new pcl::PointCloud<pcl::PointXYZ>}; ///< transformed depth cam data in Point Cloud 1


    float CalculateYawAngle();                              ///< calculates yaw angle for drone to face the target

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
    ros::Subscriber target_position_sub;                ///< target position relative to drone origin
    ros::Subscriber target_gps_sub;                     ///< target gps


    ///< Depth cam subscriber and callback
    // ros::Subscriber depth_cam_sub; ///< Transformed Depth cam subscriber in PointCloud2
    // void depth_cam_cb(const sensor_msgs::PointCloud2ConstPtr& pc2); ///< Callback for transformed depth cam data

    void target_gps_cb(const sensor_msgs::NavSatFix::ConstPtr& msg);                        ///< Callback for target gps
    void target_position_relative_cb(const geometry_msgs::PointStamped::ConstPtr& msg);     ///< Callback for target-drone relative xyz
    void target_position_cb(const geometry_msgs::PointStamped::ConstPtr& msg);              ///< Callback for target xyz from drone origin
    void altitude_cb(const mavros_msgs::Altitude::ConstPtr& msg);
    void heading_cb(const std_msgs::Float64::ConstPtr& msg); 
    void gps_cb(const sensor_msgs::NavSatFix::ConstPtr& msg);
    void imu_cb(const sensor_msgs::Imu::ConstPtr& msg);
    void pose_cb(const geometry_msgs::PoseStamped::ConstPtr &msg);
    void velocity_cb(const geometry_msgs::TwistStamped::ConstPtr &msg);
    void lidar_cb(const sensor_msgs::LaserScan::ConstPtr &msg);
    std::string get_log_name();

    boost::circular_buffer<float> yaw_angle_buffer = boost::circular_buffer<float>(3);      ///< Circular buffer for yaw angle to target
};

#endif /* DATA_H */