#include "Three_d_cam.h"

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


typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

const double pi = 3.1415926535897;

// * * * * * * * * * * * * * * * * Constructors  * * * * * * * * * * * * * * //

Three_d_cam::Three_d_cam():roll(0),pitch(0),yaw(0){

	//Topic you want to publish
  pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/camera/depth/points_pc2_voxel_filtered", 1000);
  pub2_= nh_.advertise<sensor_msgs::PointCloud2>("/camera/depth/points_pc2_passthrough_filtered", 1000);
  pub3_= nh_.advertise<sensor_msgs::PointCloud2>("/camera/depth/points_pc2_statistical_outlier_filtered", 1000);
  pub4_= nh_.advertise<sensor_msgs::PointCloud2>("/camera/depth/points_transformed", 1000);

  //Topic you want to subscribe
  sub_ = nh_.subscribe("/camera/depth/points", 1000, &Three_d_cam::callback, this);
  sub2_= nh_.subscribe("mavros/imu/data", 1000, &Three_d_cam::imuCallback, this);
    
}

// * * * * * * * * * * * * * * * * Member Functions  * * * * * * * * * * * * * * //

void Three_d_cam::callback(const sensor_msgs::PointCloud2ConstPtr& pc2){

  sensor_msgs::PointCloud2 pc2_voxel_filtered;
  sensor_msgs::PointCloud2 pc2_passthrough_filtered;
  sensor_msgs::PointCloud2 pc2_statistical_outlier_filtered;
  sensor_msgs::PointCloud2 pc2_transformed;
    
  cloud_voxel_filter(*pc2, pc2_voxel_filtered);
  pass_through_filter(*pc2, pc2_passthrough_filtered);
  statistical_outlier_filter(*pc2, pc2_statistical_outlier_filtered);
  //transform_frame(roll, pitch, yaw, *pc2, pc2_transformed);
  transform_frame(roll, pitch, yaw, pc2_voxel_filtered, pc2_transformed);


    
  pub_.publish(pc2_voxel_filtered);
  pub2_.publish(pc2_passthrough_filtered);
  pub3_.publish(pc2_statistical_outlier_filtered);
  pub4_.publish(pc2_transformed);

  // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  // pcl::fromROSMsg(pc2_transformed, *cloud);
  // std::cout<<"z: "<<cloud->points[0].z<<std::endl;

}



void Three_d_cam::imuCallback(const sensor_msgs::Imu::ConstPtr& msg){

  Eigen::Quaterniond q;
  q.x()= msg->orientation.x;
  q.y()= msg->orientation.y;
  q.z()= msg->orientation.z;
  q.w()= msg->orientation.w;

  to_euler_angle(q, roll, pitch, yaw);

}

void Three_d_cam::transform_frame(const double& roll, const double& pitch, const double& yaw, const sensor_msgs::PointCloud2 &cloud_msg, sensor_msgs::PointCloud2& pc2_tf){

	pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
  pcl::fromROSMsg(cloud_msg, *source_cloud);


  /*  METHOD #2: Using a Affine3f
  This method is easier and less error prone
  */
  Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();

  // Define a translation of 2.5 meters on the x axis.
  //transform_2.translation() << 2.5, 0.0, 0.0;

  //camera reference is x,y,z => right, down, front
  //drone imu reference is x,y,z=> right, front, up
  //transform from camera reference to drone imu body reference
  transform_2.translation() << 0.0, 0.0, 0.0;
  transform_2.rotate (Eigen::AngleAxisf (pi/2, Eigen::Vector3f::UnitY()));
  transform_2.rotate (Eigen::AngleAxisf (-pi/2, Eigen::Vector3f::UnitZ())); //now in drone imu body axis

  //Now correct frame to the frame where z is upwards and parallel to gravity, and xy parallel to surface of earth
  transform_2.rotate (Eigen::AngleAxisf (pitch, Eigen::Vector3f::UnitY()));
  transform_2.rotate (Eigen::AngleAxisf (roll, Eigen::Vector3f::UnitX())); //now in drone imu body axis
 
  // Executing the transformation
  pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
  
  pcl::transformPointCloud (*source_cloud, *transformed_cloud, transform_2);
  pcl::toROSMsg(*transformed_cloud, pc2_tf);

  ROS_INFO("roll, pitch, yaw= %f, %f, %f", roll/3.1415*180, pitch/3.1415*180, yaw/3.1415*180);

}

	
void Three_d_cam::to_euler_angle(const Eigen::Quaterniond& q, double& roll, double& pitch, double& yaw){

	// roll (x-axis rotation)
	double sinr_cosp = +2.0 * (q.w() * q.x() + q.y() * q.z());
	double cosr_cosp = +1.0 - 2.0 * (q.x() * q.x() + q.y() * q.y());
	roll = atan2(sinr_cosp, cosr_cosp);

	// pitch (y-axis rotation)
	double sinp = +2.0 * (q.w() * q.y() - q.z() * q.x());
	if (fabs(sinp) >= 1)
		pitch = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
	else
		pitch = asin(sinp);

	// yaw (z-axis rotation)
	double siny_cosp = +2.0 * (q.w() * q.z() + q.x() * q.y());
	double cosy_cosp = +1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z());  
	yaw = atan2(siny_cosp, cosy_cosp);

}


void Three_d_cam::cloud_voxel_filter(const sensor_msgs::PointCloud2& cloud_msg, sensor_msgs::PointCloud2& pc2_filtered){

	// Container for original & filtered data
  pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2; 
  pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
  pcl::PCLPointCloud2 cloud_filtered;
  //sensor_msgs::PointCloud2 pc2_filtered;

  // Convert to PCL data type
  pcl_conversions::toPCL(cloud_msg, *cloud);

  // Perform the actual filtering
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  sor.setInputCloud (cloudPtr);
  sor.setLeafSize (0.1, 0.1, 0.1);
  sor.filter (cloud_filtered);

  // Convert to ROS data type
  pcl_conversions::fromPCL(cloud_filtered, pc2_filtered);
  //sensor_msgs::convertPointCloud2ToPointCloud(pc2_filtered,pc_filtered);

}

	
void Three_d_cam::pass_through_filter(const sensor_msgs::PointCloud2& cloud_msg, sensor_msgs::PointCloud2& pc2_filtered){


	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_pcl_pc(new pcl::PointCloud<pcl::PointXYZ>);
  sensor_msgs::PointCloud cloud_msg_pc;
  pcl::PCLPointCloud2 pcl_pc2;

  //convert from sensor_msgs::PointCloud2 to PCL PointCloud2
  pcl_conversions::toPCL(cloud_msg,pcl_pc2);

  //convert to PCL pointcloud template
  pcl::fromPCLPointCloud2(pcl_pc2,*cloud_pcl_pc);
  	


  //filter parameters
  pcl::PassThrough<pcl::PointXYZ> ptfilter (true); // Initializing with true will allow us to extract the removed indices
	ptfilter.setInputCloud (cloud_pcl_pc);
	ptfilter.setFilterFieldName ("x");
	ptfilter.setFilterLimits (0.0, 0.5);
	pcl::PointCloud<pcl::PointXYZ>::Ptr indices_x (new pcl::PointCloud<pcl::PointXYZ>);
	ptfilter.filter (*indices_x);
	// The indices_x array indexes all points of cloud_in that have x between 0.0 and 1000.0
	
	// The indices_rem array indexes all points of cloud_in that have x smaller than 0.0 or larger than 1000.0
	// and also indexes all non-finite points of cloud_in
	//ptfilter.setIndices (indices_x);
	ptfilter.setFilterFieldName ("z");
	ptfilter.setFilterLimits (0, 1);

	pcl::PointCloud<pcl::PointXYZ>::Ptr indices_xz (new pcl::PointCloud<pcl::PointXYZ>);
	ptfilter.filter (*indices_xz);
	// The indices_xz array indexes all points of cloud_in that have x between 0.0 and 1000.0 and z larger than 10.0 or smaller than -10.0
	//ptfilter.setIndices (indices_xz);

	pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_pc_filtered(new pcl::PointCloud<pcl::PointXYZ>);
	ptfilter.filter (*pcl_pc_filtered);
	// The resulting cloud_out contains all points of cloud_in that are finite and have:
	// x between 0.0 and 1000.0, z larger than 10.0 or smaller than -10.0 and intensity smaller than 0.5.

	pcl::toROSMsg(*pcl_pc_filtered, pc2_filtered);

}

	
void Three_d_cam::statistical_outlier_filter(const sensor_msgs::PointCloud2& cloud_msg, sensor_msgs::PointCloud2& pc2_filtered){

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

  //convert form
  pcl::fromROSMsg(cloud_msg, *cloud);

  // Create the filtering object
	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
	sor.setInputCloud (cloud);
	sor.setMeanK (50);
	sor.setStddevMulThresh (1.0);
	sor.filter (*cloud_filtered);

	// std::cerr << "Cloud after filtering: " << std::endl;
	// std::cerr << *cloud_filtered << std::endl;

	// pcl::PCDWriter writer;
	// writer.write<pcl::PointXYZ> ("table_scene_lms400_inliers.pcd", *cloud_filtered, false);

	// sor.setNegative (true);
	// sor.filter (*cloud_filtered);
	// writer.write<pcl::PointXYZ> ("table_scene_lms400_outliers.pcd", *cloud_filtered, false);

	//convert form
	pcl::toROSMsg(*cloud_filtered, pc2_filtered);

}






