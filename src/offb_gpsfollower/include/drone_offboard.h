#pragma once

#include <ros/ros.h>
#include <mavros_msgs/HomePosition.h>
#include <mavros_msgs/Altitude.h>
#include <sensor_msgs/NavSatFix.h>

class Drone
{
public:
  Drone();
  ~Drone();

  // Actions
  bool arm();
  bool takeoff();
  bool land();
  bool setOffboardMode();

  // Offboard Velocity Control
  void FlyToGlobalPosTarget();
  void read_gps_cb(const sensor_msgs::NavSatFix::ConstPtr& msg);
  ros::Rate rate_ = ros::Rate(20.0);

private:
  void getHomeGeoPoint();
  void getAltitude();
  void setHomeGeoPointCB(const mavros_msgs::HomePositionConstPtr& home);
  void getAltitudeCB(const mavros_msgs::AltitudeConstPtr& altitude);
  void getTargetGPS_CB(const sensor_msgs::NavSatFix::ConstPtr& msg);
  float toRadFromDeg(float deg);

  ros::NodeHandle nh_;
  mavros_msgs::HomePosition home_{};
  mavros_msgs::Altitude altitude_{};
  sensor_msgs::NavSatFix TargetGPS_{};
  ros::Publisher set_vel_pub_;
  ros::Publisher set_global_pub_;
  ros::Subscriber read_gps_sub_;
  ros::ServiceClient takeoff_client_;
  ros::ServiceClient set_mode_client_;
  ros::ServiceClient land_client_;
  bool home_set_ = false;
  bool altitude_received_ = false;
  bool target_gps_received_ = false;
  float altitude_in_amsl_ = 0.0f;
  double target_longitude_;
  double target_latitude_;
  float target_altitude_;
};