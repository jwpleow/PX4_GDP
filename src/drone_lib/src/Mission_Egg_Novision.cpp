#include "headers/gdpdrone.h"


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
    float altitude = 4;
    int time_takeoff = 80;
    drone.Commands.request_Takeoff(altitude, time_takeoff);


 ///<------ STEP 1  ---- EGG COLLECTION ---- NO VISION ---- FLIES TO (-2,-2) ( see where this lands the first time, and place first ARtag & egg there)



    ROS_INFO("Moving to egg location");
    for (int count = 1; count < 51; count++ )
    {
        drone.Commands.move_Position_Local(0.0f, 0.0f, 0.0f, 225.0f, "LOCAL_OFFSET", count);
        ros::spinOnce();
        rate.sleep();
    }



    for (int count = 1; count < 61; count++)
    {
        drone.Commands.move_Position_Local(-2.0f, -2.0f, 0.0f, 225.0f, "LOCAL_OFFSET", count);
        ros::spinOnce();
        rate.sleep();
    }

    // Land and disarm
    ROS_INFO("Landing Now");
    drone.Commands.request_LandingAuto();

    return 0;
}