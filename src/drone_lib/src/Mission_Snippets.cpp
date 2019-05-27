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
    float altitude = 0.35;
    int time_takeoff = 250;
    drone.Commands.request_Takeoff(altitude, time_takeoff);

    // Go one meter up and stay there. Total time 10 seconds
    ROS_INFO("First Command");
    for (int count = 1; count < 250; count++)
    {
        drone.Commands.move_Position_Local(0, 0, 1, 90, "BODY");
        ros::spinOnce();
        rate.sleep();
    }

    // Accelerate at 1 m/ss Northwards for 10 seconds
    ROS_INFO("Second Command");
    drone.Commands.reset_Velocities(); // For God's Sake, don't forget this!
    for (int count = 1; count < 250; count++)
    {
        drone.Commands.move_Acceleration_Local_Trick(1, 0, 0, "BODY", loop_rate);
        ros::spinOnce();
        rate.sleep();
    }

    // Accelerate at 1 m/ss Fowards for 10 seconds (body frame)
    ROS_INFO("Third Command");
    drone.Commands.reset_Velocities(); // For God's Sake, don't forget this!
    for (int count = 1; count < 250; count++)
    {
        drone.Commands.move_Acceleration_Local_Trick(1, 0, 0, "LOCAL", loop_rate);
        ros::spinOnce();
        rate.sleep();
    }

    ROS_INFO("Fourth Command, go to GPS Position");
    for (int count = 1; count < 200; count++)
    {
        drone.Commands.move_Position_Global(47.39770, 8.5456, 545, 90, "BODY"); // Here BODY will only be useful for the yaw
        ros::spinOnce();
        rate.sleep();
    }

    ROS_INFO("Fifth Command, Local Velocity Command");
    for (int count = 1; count < 100; count++)
    {
        drone.Commands.move_Velocity_Local(1, 1, 1, 0, "LOCAL");
        ros::spinOnce();
        rate.sleep();
    }

    // Land and disarm
    drone.Commands.request_LandingAuto();

    // Exit
    return 0;
}