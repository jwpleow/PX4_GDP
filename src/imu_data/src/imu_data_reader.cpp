 #include "ros/ros.h"
 #include "sensor_msgs/Imu.h"
 #include <fstream>
 #include <iostream>

 
 void chatterCallback(const sensor_msgs::Imu::ConstPtr& msg){
     ROS_INFO("\nlinear acceleration\
                 \nx:[%f]\ny:[%f]\nz:[%f]", msg->linear_acceleration.x,
                 msg->linear_acceleration.y, msg->linear_acceleration.z);

// outputFile << msg->linear_acceleration.x << ' ' 
// << msg->linear_acceleration.y << ' ' << msg->linear_acceleration.z << std::endl;



}

int main(int argc, char **argv){
    ros::init(argc, argv, "imu_data_reader");
    ros::NodeHandle n;


    // // // open file
    // global std::ofstream outputFile("/home/jiaweikhor/mydata.txt");


    ros::Subscriber sub = n.subscribe("/mavros/imu/data", 1000, chatterCallback);






    ros::spin();

    // // close file
    // outputFile.close();

    return 0;
}