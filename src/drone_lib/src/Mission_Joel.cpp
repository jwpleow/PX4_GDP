#include "headers/gdpdrone.h"
#include <iostream>

int main(int argc, char **argv)
{
    // Initialise node
    ros::init(argc, argv, "joel_node");

    // Create drone object, this sets everything up
    GDPdrone drone;

    // Set the rate. Default working frequency is 25 Hz
    float loop_rate = 10.0f;
    ros::Rate rate = ros::Rate(loop_rate);

    // Initialise and Arm
    drone.Commands.await_Connection();
    drone.Commands.set_Offboard();
    drone.Commands.set_Armed();

    // MISSION STARTS HERE:
    // Request takeoff at 1m altitude. At 25Hz = 10 seconds
    float altitude = 5.0f;
    int time_takeoff = 125;
    drone.Commands.request_Takeoff(altitude, time_takeoff);

// bool landingpaddetected = false;
// while(!landingpaddetected){

    // ROS_INFO("Track Ambulance");
    // for (int count = 1; count < 20000; count++)
    // {   
    // //     // // Use the command below to move to setpoint at max speed:
    //     // drone.Commands.move_Position_Global(drone.Data.target_gps.latitude, drone.Data.target_gps.longitude, drone.Data.target_gps.altitude + 5.0f, drone.Data.CalculateYawAngle(), "LOCAL");
    // //     ROS_INFO("Angle: %f", drone.Data.CalculateYawAngle());


    //     drone.Commands.move_Velocity_Local(2.0, drone.Data.CalculateYawAngle(), "LOCAL_OFFSET");
    //     ros::spinOnce();
    //     rate.sleep();
    // }

// ros::spinOnce();
// drone.Data.rate.sleep();
// } ///< landing pad detected



    // ROS_INFO("Test velocity handover to accel - velocity for 10s");
    // drone.Commands.Initialise_Velocity_for_AccelCommands(2.0f, 0.0f, 0.0f);
    // for (int count = 1; count < 200; count++){
    //     drone.Commands.move_Velocity_Local(2.0f, 0.0f, 0.0f, 0.0f, "LOCAL_OFFSET");
    //     ros::spinOnce();
    //     rate.sleep();
    // }
    // ROS_INFO("Switch to accel");
    // for (int count = 1; count < 200; count++){
    //     drone.Commands.move_Acceleration_Local_Trick(0.2f, 0.0f, 0.0f, "LOCAL_OFFSET", loop_rate);
    //     ros::spinOnce();
    //     rate.sleep();
    // }


ROS_INFO("x = 2");
for (int count = 1; count < 100; count++){
drone.Commands.move_Position_Local(1.0f, 0.0f, 0.0f, 0.0f, "BODY_OFFSET");
        ros::spinOnce();
        rate.sleep();

}

ROS_INFO("x = 1");
for (int count = 1; count < 100; count++){
drone.Commands.move_Position_Local(1.0f, 0.0f, 0.0f, 0.0f, "BODY_OFFSET");
        ros::spinOnce();
        rate.sleep();

}



ROS_INFO("y");
for (int count = 1; count < 200; count++){
drone.Commands.move_Position_Local(0.0f, 2.0f, 0.0f, 0.0f, "BODY_OFFSET");
        ros::spinOnce();
        rate.sleep();

}

ROS_INFO("z");
for (int count = 1; count < 200; count++){
drone.Commands.move_Position_Local(0.0f, 0.0f, 0.0f, 0.0f, "BODY_OFFSET");
        ros::spinOnce();
        rate.sleep();

}

ROS_INFO("turn");
for (int count = 1; count < 200; count++){
drone.Commands.move_Position_Local(0.0f, 0.0f, 0.0f, 5.0f, "BODY_OFFSET");
        ros::spinOnce();
        rate.sleep();

}

ROS_INFO("x");
for (int count = 1; count < 200; count++){
drone.Commands.move_Position_Local(2.0f, 0.0f, 0.0f, 0.0f, "BODY_OFFSET");
        ros::spinOnce();
        rate.sleep();

}

ROS_INFO("y");
for (int count = 1; count < 200; count++){
drone.Commands.move_Position_Local(0.0f, 2.0f, altitude, 0.0f, "BODY_OFFSET");
        ros::spinOnce();
        rate.sleep();

}

ROS_INFO("z");
for (int count = 1; count < 200; count++){
drone.Commands.move_Position_Local(0.0f, 0.0f, 2.0f, 0.0f, "BODY_OFFSET");
        ros::spinOnce();
        rate.sleep();

}
    // Land and disarm
    drone.Commands.request_LandingAuto();

    // Exit
    return 0;
}