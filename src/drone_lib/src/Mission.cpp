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
    float altitude = 3;
    int time_takeoff = 300;
    drone.Commands.request_Takeoff(altitude, time_takeoff);

    /*
    // Go one meter up and stay there. Total time 10 seconds
    ROS_INFO("Goto Command");
    for (int count = 1; count < 125; count++)
    {
        drone.Commands.move_Position_Local(-5, 5, 2, 0, "LOCAL");
        ros::spinOnce();
        rate.sleep();
    }


    ROS_INFO("First Command");
    for (int count = 1; count < 125; count++)
    {
        drone.Commands.move_Position_Local(2, 0, 0, 0, "BODY_OFFSET");
        ros::spinOnce();
        rate.sleep();
    }

    ROS_INFO("Second Command");
    for (int count = 1; count < 125; count++)
    {
        drone.Commands.move_Position_Local(0, 2, 0, 0, "BODY_OFFSET");
        ros::spinOnce();
        rate.sleep();
    }

    
    ROS_INFO("Third Command");
    for (int count = 1; count < 125; count++)
    {
        drone.Commands.move_Position_Local(-2, 0, 0, 0, "BODY_OFFSET");
        ros::spinOnce();
        rate.sleep();
    }

    ROS_INFO("Fourth Command");
    for (int count = 1; count < 125; count++)
    {
        drone.Commands.move_Position_Local(0, -2, 0, 0, "BODY_OFFSET");
        ros::spinOnce();
        rate.sleep();
    }

    ROS_INFO("Fifth Command");
    for (int count = 1; cyount < 125; count++)
    {
        drone.Commands.move_Velocity_Local(0, 2, altitude, 0, "BODY_OFFSET");
        ros::spinOnce();
        rate.sleep();
    }
    */

    // Land and disarm
    drone.Commands.request_LandingAuto();

    // Exit
    return 0;
}