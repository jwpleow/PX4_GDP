#ifndef GRABBER_H
#define GRABBER_H

#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float64.h>

class grabber
{
public:
    grabber();
    std_msgs::Float64 force_sensor_data;

    ros::NodeHandle nh;
    ros::Subscriber force_sensor_sub;
    ros::Publisher pump_on;
    ros::Publisher pump_off;
    ros::Publisher servo_on;
    ros::Publisher servo_off;

    void turn_on_pump();
    void turn_off_pump();

private:
    void force_sensor_cb(const std_msgs::Float64::ConstPtr &msg);
};

#endif /* GRABBER_H */