#include "headers/gdpdrone.h"

int main(int argc, char **argv)
{
    // Initialise node
    ros::init(argc, argv, "drone_node");

    // Create drone object, this sets everything up
    GDPdrone drone;

    // Set the rate. Default working frequency is 25 Hz
    float loop_rate = 25.0;
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




    ROS_INFO("Track Ambulance");
    for (int count = 1; count < 10000; count++)
    {
        drone.Commands.move_Position_Global(drone.Data.gps_raw.latitude, drone.Data.gps_raw.longitude, drone.Data.gps_raw.altitude + 5.0f, drone.Data.CalculateYawAngle(), "BODY");
        ros::spinOnce();
        rate.sleep();
    }

    // Land and disarm
    drone.Commands.request_LandingAuto();

    // Exit
    return 0;
}