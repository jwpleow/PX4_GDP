 #include "ros/ros.h"
 #include <fstream>
 #include <iostream>
 #include "sensor_msgs/PointCloud2.h"
#include <geometry_msgs/Point.h>
 #include <vector>
// #include "p.h"
//  #include <point_cloud2_iterator.h>





// ros::Publisher depthcam_data;





std::vector<float> RangeData(9830400);

void depthcam_cb(const sensor_msgs::PointCloud2ConstPtr& msg){


 	ROS_INFO("\nHeight is: %i  ", msg->height);
 	ROS_INFO("\nWidth is: %i  ", msg->width);
	ROS_INFO("\ndatalength is: %i ", (msg->data.end()-msg->data.begin()));

	for(int i = 0; i < 9803400; ++i)
	std::cout << msg->data[i];
 // std::copy(msg->data.begin(),msg->data.end(),RangeData.begin());

// for (int i=0; i < 100;i++)
// std::cout << RangeData[i] << ' ';
 	// ROS_INFO("\n is:[%i] ", msg->data());

// sensor_msgs::LaserScan laser_msg;
// laser_msg.header.stamp = ros::Time(_msg—>time().sec(), _msg—>time().nsec());


// // Publish in ROS topic
// tera_2_pub_rplidar.publish(laser_msg);

	     
}



  int main(int argc, char **argv)
  {

  ros::init(argc, argv, "depthcam_listener");
  
  ros::NodeHandle n;
 
 
  ros::Subscriber depthcam_sub = n.subscribe<sensor_msgs::PointCloud2>("/camera/depth/points", 1000, depthcam_cb);
 
  
  ros::spin();
 
  return 0;
  }






// /**
//     Function to convert 2D pixel point to 3D point by extracting point
//     from PointCloud2 corresponding to input pixel coordinate. This function
//     can be used to get the X,Y,Z coordinates of a feature using an 
//     RGBD camera, e.g., Kinect.
//     */
//     void pixelTo3DPoint(const sensor_msgs::PointCloud2 pCloud, const int u, const int v, geometry_msgs::Point &p)
//     {
//       // get width and height of 2D point cloud data
//       int width = pCloud.width;
//       int height = pCloud.height;

//       // Convert from u (column / width), v (row/height) to position in array
//       // where X,Y,Z data starts
//       int arrayPosition = v*pCloud.row_step + u*pCloud.point_step;

//       // compute position in array where x,y,z data start
//       int arrayPosX = arrayPosition + pCloud.fields[0].offset; // X has an offset of 0
//       int arrayPosY = arrayPosition + pCloud.fields[1].offset; // Y has an offset of 4
//       int arrayPosZ = arrayPosition + pCloud.fields[2].offset; // Z has an offset of 8

//       float X = 0.0;
//       float Y = 0.0;
//       float Z = 0.0;

//       memcpy(&X, &pCloud.data[arrayPosX], sizeof(float));
//       memcpy(&Y, &pCloud.data[arrayPosY], sizeof(float));
//       memcpy(&Z, &pCloud.data[arrayPosZ], sizeof(float));

//      // put data into the point p
//       p.x = X;
//       p.y = Y;
//       p.z = Z;

//     }