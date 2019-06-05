#include "headers/gdpdrone.h"
#include <iostream>

int main(int argc, char **argv)
{
    // Initialise node
    ros::init(argc, argv, "gerald_node");

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

    ROS_INFO("Track Ambulance");
    for (int count = 1; count < 200; count++)
    {
        float yaw_angle = drone.Data.CalculateYawAngle();
        std::cout << "Yaw angle is: " << yaw_angle << " deg" << std::endl;
        // drone.Commands.move_Position_Global(drone.Data.gps_raw.latitude, drone.Data.gps_raw.longitude, drone.Data.gps_raw.altitude + 5.0f, yaw_angle, "BODY");
        drone.Commands.move_Velocity_Local_Gerald(1.50, yaw_angle, "BODY");
        ros::spinOnce();
        rate.sleep();
    }



// ros::spinOnce();
// drone.Data.rate.sleep();
// } ///< landing pad detected



    // Land and disarm
    drone.Commands.request_LandingAuto();

    // Exit
    return 0;
}
