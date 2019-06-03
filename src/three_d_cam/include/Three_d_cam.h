#include <ros/console.h>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <geometry_msgs/TransformStamped.h>
#include "sensor_msgs/Imu.h"
#include <Eigen/Geometry>
#include <math.h>



#ifndef CLASS_THREE_D_CAM
#define CLASS_THREE_D_CAM

class Three_d_cam {


public:

	// * * * * * * * * * * * * * * * * Constructors  * * * * * * * * * * * * * * //
	
	Three_d_cam();


	// * * * * * * * * * * * * * * * * Member Functions  * * * * * * * * * * * * * * //

	void callback(const sensor_msgs::PointCloud2ConstPtr& pc2);

	void imuCallback(const sensor_msgs::Imu::ConstPtr& msg);

	void transform_frame(const double& roll, const double& pitch, const double& yaw, const sensor_msgs::PointCloud2 &cloud_msg, sensor_msgs::PointCloud2& pc2_tf);

	static void to_euler_angle(const Eigen::Quaterniond& q, double& roll, double& pitch, double& yaw);

	void cloud_voxel_filter(const sensor_msgs::PointCloud2& cloud_msg, sensor_msgs::PointCloud2& pc2_filtered);

	void pass_through_filter(const sensor_msgs::PointCloud2& cloud_msg, sensor_msgs::PointCloud2& pc2_filtered);

	void statistical_outlier_filter(const sensor_msgs::PointCloud2& cloud_msg, sensor_msgs::PointCloud2& pc2_filtered);

private:

	ros::NodeHandle nh_; 
	ros::Publisher pub_;
	ros::Publisher pub2_;
	ros::Publisher pub3_;
	ros::Publisher pub4_;
	ros::Subscriber sub_;
	ros::Subscriber sub2_;
	double roll;
	double pitch;
	double yaw;


};



#endif