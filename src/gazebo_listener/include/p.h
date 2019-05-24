#pragma once

// class p to hold x y z data of thing
 #include "ros/ros.h"
 #include <fstream>
 #include <iostream>
 #include "sensor_msgs/PointCloud2.h"
#include <geometry_msgs/Point.h>
 #include <vector>
#include "p.h"


class p
{
public:
  p();
  ~p();


private:
  
  ros::NodeHandle nh_;
  ros::Publisher set_vel_pub_;
  ros::ServiceClient takeoff_client_;
  ros::Rate rate_ = ros::Rate(20.0);
};








