#include "headers/functions.h"

functions::functions(){}

const float functions::pi = 3.1415926535f;

double functions::DegToRad(float x)
{
    float ans = x / 180 * pi;
    return ans;
}

double functions::RadToDeg(float x)
{
    float ans = x * 180 / pi;
    return ans;
}

// Simplify pose creation
// Overload 1 - No orientation
geometry_msgs::PoseStamped functions::make_pose(float _x, float _y, float _z)
{
    geometry_msgs::PoseStamped poseObj;
    poseObj.header.stamp = ros::Time::now();
    poseObj.pose.position.x = _x;
    poseObj.pose.position.y = _y;
    poseObj.pose.position.z = _z;
    poseObj.pose.orientation.x = 0;
    poseObj.pose.orientation.y = 0;
    poseObj.pose.orientation.z = 0;
    poseObj.pose.orientation.w = 0;
    return poseObj;
}

// Overload 2 - Euler Quaternion
geometry_msgs::PoseStamped functions::make_pose(float _x, float _y, float _z, float _qx, float _qy, float _qz, float _theta)
{
    geometry_msgs::PoseStamped poseObj = make_pose(_x, _y, _z);
    poseObj.pose.orientation.x = _qx;
    poseObj.pose.orientation.y = _qy;
    poseObj.pose.orientation.z = _qz;
    poseObj.pose.orientation.w = _theta;
    return poseObj;
}

// Set velocity message
geometry_msgs::Twist functions::make_twist(float _x, float _y, float _z, float _ax, float _ay, float _az)
{
    geometry_msgs::Twist velocityObj;
    velocityObj.linear.x = _x;
    velocityObj.linear.y = _y;
    velocityObj.linear.z = _z;
    velocityObj.angular.x = _ax;
    velocityObj.angular.y = _ay;
    velocityObj.angular.z = _az;
    return velocityObj;
}

geometry_msgs::Vector3Stamped functions::make_acceleration(float _x, float _y, float _z)
{
    geometry_msgs::Vector3Stamped accelerationObj;
    accelerationObj.header.stamp = ros::Time::now();
    accelerationObj.vector.x = _x;
    accelerationObj.vector.y = _y;
    accelerationObj.vector.z = _z;
    return accelerationObj;
}