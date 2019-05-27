/**
 * Adapted from geodetic_utils library for only GPS -> NED data
 * */

#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/PointStamped.h>
#include <follower/geodetic_conv.hpp>
#include <std_msgs/Float64.h>

geodetic_converter::GeodeticConverter g_geodetic_converter;

ros::Publisher g_gps_position_pub;

bool g_trust_gps;
std::string g_frame_id;

void gps_callback(const sensor_msgs::NavSatFix::ConstPtr& msg) {
    if (msg->status.status < sensor_msgs::NavSatStatus::STATUS_FIX) {
        ROS_WARN_STREAM_THROTTLE(1, "No GPS fix");
        return;
    }

    if (!g_geodetic_converter.isInitialised()) {
        ROS_WARN_STREAM_THROTTLE(1, "No GPS reference point set, not publishing");
        return;
    }

    double x, y, z;
    g_geodetic_converter.geodetic2Enu(msg->latitude, msg->longitude, msg->altitude, &x, &y, &z);

    // Fill up position message
    geometry_msgs::PointStampedPtr position_msg(
            new geometry_msgs::PointStamped);
    position_msg->header = msg->header;
    position_msg->header.frame_id = g_frame_id;
    position_msg->point.x = x;
    position_msg->point.y = y;
    position_msg->point.z = z;

    g_gps_position_pub.publish(position_msg);
}




///< get reference LLA from drone
double latitude, longitude, altitude;
bool connection = false;
sensor_msgs::NavSatFix DroneGPS_{};

void getDroneGPS_CB(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
  DroneGPS_ = *msg;
  connection = true;
  longitude = DroneGPS_.longitude;
  latitude = DroneGPS_.latitude;
  altitude = DroneGPS_.altitude;
}





int main(int argc, char **argv) {
  ros::init(argc, argv, "gps_wrtdrone_conversion_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  ros::Rate rate(10);

  ///< Subscribe to Drone GPS Data for reference LLA
  ros::Subscriber read_gps_sub_ = nh.subscribe<sensor_msgs::NavSatFix>("/mavros/global_position/global",10, getDroneGPS_CB);
  
  if (!connection) ROS_INFO("Waiting for GPS reference parameters...");
  while(!connection){
    ros::spinOnce();
        rate.sleep();
  }

  
  // Specify whether covariances should be set manually or from GPS
  ros::param::param("~trust_gps", g_trust_gps, false);

  // Get manual parameters
  ros::param::param<std::string>("~frame_id",
                                 g_frame_id, "world");

  // initialise GPS reference parameters
  g_geodetic_converter.initialiseReference(latitude, longitude, altitude);

  // Show reference point
  double initial_latitude, initial_longitude, initial_altitude;
  g_geodetic_converter.getReference(&initial_latitude, &initial_longitude,
                                    &initial_altitude);
  ROS_INFO("GPS reference initialized correctly %f, %f, %f", initial_latitude,
           initial_longitude, initial_altitude);

  // Initialize publisher
  g_gps_position_pub =
      nh.advertise<geometry_msgs::PointStamped>("gps_wrtdrone_position", 1);

  // Subscribe GPS Fix and convert in callback
  ros::Subscriber gps_sub = nh.subscribe("/android/fix", 1, &gps_callback);
  ros::spin();
}
