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

#include <Three_d_cam.h>


int main(int argc, char **argv)
{

  //Initiate ROS
  ros::init(argc, argv, "pointcloud2_to_pointcloud");
  ros::NodeHandle nh;

  //Create an object of class Three_d_cam that will take care of everything
  Three_d_cam SAPObject;
  
  ros::Rate r(10); // 10 hz
  while (ros::ok())
  {
  	
  	ros::spinOnce();
  	r.sleep();

  }

  // Three_d_cam x;
  // ros::spin();

}

  

