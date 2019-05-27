#ifndef FUNCTIONS_H
#define FUNCTIONS_H

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3Stamped.h>  

class functions
{
public:
    static const float pi;
    functions();
    static double DegToRad(float x);
    static double RadToDeg(float x);
    static geometry_msgs::PoseStamped make_pose(float _x, float _y, float _z);
    static geometry_msgs::PoseStamped make_pose(float _x, float _y, float _z, float _qx, float _qy, float _qz, float _theta);
    static geometry_msgs::Twist make_twist(float _x, float _y, float _z, float _ax, float _ay, float _az);
    static geometry_msgs::Vector3Stamped make_acceleration(float _x, float _y, float _z);
};

#endif /* FUNCTIONS_H */