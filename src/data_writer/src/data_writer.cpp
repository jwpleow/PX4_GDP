#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/PointStamped.h"
#include <fstream>
#include <iostream>

bool connection = 0;
// open file stream
std::ofstream outputtargetpos("/home/khorjiawei/bagfiles/TargetWRTdronedata.txt");

void target_wrtdrone_cb(const geometry_msgs::PointStamped::ConstPtr &msg)
{

    outputtargetpos << msg->header.stamp << ' ' << msg->point.x << ' '
                    << msg->point.y << ' ' << msg->point.z << std::endl;

connection = 1;
}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "data_writer");
    ros::NodeHandle n;
    ros::Rate rate(10);




    ros::Subscriber sub = n.subscribe("/filtered_target_wrtdrone_position", 1, target_wrtdrone_cb);

    while(connection == 0)
    {
        ROS_INFO("Waiting For Connection");
        ros::spinOnce();
        rate.sleep();
    }

    ROS_INFO("Connected");





    ros::spin();

    // close files
    outputtargetpos.close();

    return 0;
}