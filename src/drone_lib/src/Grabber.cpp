#include "headers/grabber.h"

grabber::grabber()
{
    pump_on = nh.advertise<std_msgs::Empty>("/serial_node/pump_on", 1);
    pump_off = nh.advertise<std_msgs::Empty>("/serial_node/pump_off", 1);
    servo_on = nh.advertise<std_msgs::Empty>("/serial_node/servo_on", 1);
    servo_off = nh.advertise<std_msgs::Empty>("/serial_node/servo_off", 1);

    // Subscribe to force sensor
    force_sensor_sub = nh.subscribe<std_msgs::Float64>("/serial_node/fsr_analog", 1, &grabber::force_sensor_cb, this);
}

void grabber::force_sensor_cb(const std_msgs::Float64::ConstPtr &msg)
{
    force_sensor_data = *msg;
}
