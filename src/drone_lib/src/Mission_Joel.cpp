#include "headers/gdpdrone.h"
#include <iostream>

int main(int argc, char **argv)
{
    // Initialise node
    ros::init(argc, argv, "joel_node");

    // Create drone object, this sets everything up
    GDPdrone drone;

    // Set the rate. Default working frequency is 25 Hz
    float loop_rate = 10.0;
    ros::Rate rate = ros::Rate(loop_rate);

    // Initialise and Arm
    drone.Commands.await_Connection();
    drone.Commands.set_Offboard();
    drone.Commands.set_Armed();

    // MISSION STARTS HERE:
    // Request takeoff at 1m altitude. At 25Hz = 10 seconds
    float altitude = 5;
    int time_takeoff = 125;
    drone.Commands.request_Takeoff(altitude, time_takeoff);

// bool landingpaddetected = false;
// while(!landingpaddetected){

    // ROS_INFO("Track Ambulance");
    // for (int count = 1; count < 20000; count++)
    // {   
    //     // // Use the command below to move to setpoint at max speed:
    //     drone.Commands.move_Position_Global(drone.Data.target_gps.latitude, drone.Data.target_gps.longitude, drone.Data.target_gps.altitude + 5.0f, 0.0f, "LOCAL");
    //     ROS_INFO("Angle: %f", drone.Data.CalculateYawAngle());
    //     // // drone.Commands.move_Velocity_Local(1.50, yaw_angle, "BODY");
    //     ros::spinOnce();
    //     rate.sleep();
    // }

// ros::spinOnce();
// drone.Data.rate.sleep();
// } ///< landing pad detected



ROS_INFO("Test acceleration");
for (int count = 1; count < 2000; count++){
drone.Commands.move_Acceleration_Local_Trick(1.0f, 0.0f, 0.0f, "LOCAL_OFFSET", loop_rate);
ros::spinOnce();
rate.sleep();
}

    // Land and disarm
    drone.Commands.request_LandingAuto();

    // Exit
    return 0;
}