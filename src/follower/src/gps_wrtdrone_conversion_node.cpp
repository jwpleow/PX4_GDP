/**
 * Adapted from geodetic_utils library for only GPS -> NED data
 * */

#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/PointStamped.h>
#include <follower/geodetic_conv.hpp>
#include <std_msgs/Float64.h>
#include <std_msgs/Header.h>

geodetic_converter::GeodeticConverter g_geodetic_converter;

ros::Publisher g_gps_position_pub;

bool g_trust_gps;
std::string g_frame_id;

    // Fill up position message
    geometry_msgs::PointStampedPtr position_msg(new geometry_msgs::PointStamped); ///< Made global to publish
    double x, y, z, targ_latitude, targ_longitude, targ_altitude; ///< global now
    std_msgs::Header targ_header;
    bool targ_connection = false;

///< Callback function to take GPS data
void gps_callback(const sensor_msgs::NavSatFix::ConstPtr& msg) {
    if (msg->status.status < sensor_msgs::NavSatStatus::STATUS_FIX) {
        ROS_WARN_STREAM_THROTTLE(1, "No GPS fix");
        return;
    }

    if (!g_geodetic_converter.isInitialised()) {
        ROS_WARN_STREAM_THROTTLE(1, "No GPS reference point set, not publishing");
        return;
    }


    targ_header = msg->header;
    targ_latitude = msg->latitude;
    targ_longitude = msg->longitude;
    targ_altitude = msg->altitude;
    targ_connection = true;
    
}

///< get reference LLA from drone
double latitude, longitude, altitude;
bool connection = false;
sensor_msgs::NavSatFix droneGPS_{};

void drone_ref_callback(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
  droneGPS_ = *msg;
  connection = true;
  longitude = droneGPS_.longitude;
  latitude = droneGPS_.latitude;
  altitude = droneGPS_.altitude;

  // initialise geodetic reference as this point (updates per subscriber Hz)
  g_geodetic_converter.initialiseReference(latitude, longitude, altitude);

    ///< do calculations and publish
    g_geodetic_converter.geodetic2Enu(targ_latitude, targ_longitude, targ_altitude, &x, &y, &z);


if (targ_connection){
    position_msg->header.frame_id = g_frame_id;
    position_msg->point.x = x;
    position_msg->point.y = y;
    position_msg->point.z = z;
    position_msg->header = targ_header;

    g_gps_position_pub.publish(position_msg);
  }
}

int main(int argc, char **argv) {
  
  ros::init(argc, argv, "gps_wrtdrone_conversion_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  ros::Rate rate(10);


  // Subscribe to Drone GPS Data for reference LLA
  ros::Subscriber read_gps_sub_ = nh.subscribe<sensor_msgs::NavSatFix>("/mavros/global_position/global", 10, &drone_ref_callback);
  
  if (!connection) ROS_INFO("Waiting for GPS reference parameters...");
  while(!connection){
    ros::spinOnce();
    rate.sleep();
  }

  ROS_INFO("GPS reference parameters retrieved.");

  // Specify whether covariances should be set manually or from GPS
  ros::param::param("~trust_gps", g_trust_gps, false);

  // Get manual parameters
  ros::param::param<std::string>("~frame_id",
                                 g_frame_id, "world");

  // Initialize publisher
  g_gps_position_pub = nh.advertise<geometry_msgs::PointStamped>("gps_wrtdrone_position", 10);

  // Subscribe GPS Fix and convert in callback
  ros::Subscriber gps_sub = nh.subscribe("/android/fix", 10, &gps_callback);





  ros::spin();
  
}
