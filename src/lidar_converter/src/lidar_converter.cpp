#include <mavros_msgs/Altitude.h>
#include <sensor_msgs/Range.h>
#include "ros/ros.h"




bool connection = 0;
ros::Publisher range_pub;

void altitude_cb(const mavros_msgs::Altitude::ConstPtr& msg){

	sensor_msgs::Range rangedata;

    rangedata.range = msg->bottom_clearance;
    rangedata.header = msg->header;
    rangedata.min_range = 0.2; //values from testing
    rangedata.max_range = 11;
    rangedata.field_of_view = 0.0523599;
    rangedata.radiation_type = 1; //1 for IR
   	connection = 1;

    range_pub.publish(rangedata);
}




int main(int argc, char **argv){
	// Initialise ROS with node name
 	ros::init(argc, argv, "teraranger_one_node");

 	ros::NodeHandle n;
 	ros::Rate rate(10);

 	// Publish to topic /teraranger_one
	range_pub = n.advertise<sensor_msgs::Range>("teraranger_one", 10);
	
  	// Subscribe to Altitude Data
	ros::Subscriber altitude_sub = n.subscribe<mavros_msgs::Altitude>("/mavros/altitude", 10, &altitude_cb);

	if (!connection) ROS_INFO("Waiting for LiDAR data...");
  	while(!connection){
    	ros::spinOnce();
    	rate.sleep();
  	}

  	ROS_INFO("Altitude Data received");

	

	

	ros::spin();

	return 0;
}
