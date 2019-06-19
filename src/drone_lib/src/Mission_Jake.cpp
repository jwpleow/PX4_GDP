#include "headers/gdpdrone.h"
#include "headers/jakelibrary.h"
#include <cmath>

int main(int argc, char **argv)
{
    // Initialise node
    ros::init(argc, argv, "jake_node");

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
    // Request takeoff at 5.77m altitude.
    float altitude = 5.77f;
    float setAltitude = 5.77f;
    int time_takeoff = 80; // 5 seconds at 10 Hz
    ROS_INFO("Setting altitude to 5.77 m");
    drone.Commands.request_Takeoff(altitude, time_takeoff);






    InitialiseJakeCode(drone.Data.target_position_relative.point.x, drone.Data.target_position_relative.point.y, drone.Data.target_position_relative.point.z);

    // Command 1, set drone velocity to the calculated initial velocity in 1 second.
    ROS_INFO("Initialising drone velocity");
    // Change this to a while loop comparing measured drone velocity and commanded drone velocity

    drone.Commands.Initialise_Velocity_for_AccelCommands(droneVel[1], droneVel[0], -droneVel[2]);

    // float initial_yaw = atan2(droneVel[0], droneVel[1]) * 180.0 / 3.14159;
    // // turn drone to point in that direction
    // ROS_INFO("Turning to direction of inital velocity: %f degrees", initial_yaw); // assume 10 degrees / second?
    // for (int count = 1; count < floor(initial_yaw / 15 * loop_rate); count++)
    // {
    //     drone.Commands.move_Position_Local(0.0f, 0.0f, 0.0f, initial_yaw, "LOCAL_OFFSET", count);
    //     ros::spinOnce();
    //     rate.sleep();
    // }


    InitialiseJakeCode(drone.Data.target_position_relative.point.x, drone.Data.target_position_relative.point.y, drone.Data.target_position_relative.point.z);

    drone.Commands.Initialise_Velocity_for_AccelCommands(droneVel[1], droneVel[0], -droneVel[2]);
    // Actual proportional navigation algorithm
    ROS_INFO("Starting proportional navigation algorithm");

    do
    {

        droneAccComp(relPos, relVel, droneAcc);
        accFix = altitudeFix(drone.Data.target_position_relative.point.z, setAltitude);
        ROS_INFO("Accelerations needed: x: %f, y: %f, z: %f", droneAcc[1], droneAcc[0], droneAcc[2]);
        drone.Commands.move_Acceleration_Local_Trick(droneAcc[1], droneAcc[0], accFix, "LOCAL_OFFSET", loop_rate);

        for (int i = 0; i < 3; ++i)
        {
            relPosOld[i] = relPos[i];
        }

        relPos[0] = drone.Data.target_position_relative.point.y;
        relPos[1] = drone.Data.target_position_relative.point.x;
        relPos[2] = 0;
        gpsdistance = norm(relPos);
        relPos[2] = drone.Data.target_position_relative.point.z;



        ROS_INFO("Distance to target: %f", gpsdistance);

        velFromGPS(relPos, relPosOld, loop_rate, relVel);


        ros::spinOnce();
        rate.sleep();



    }
    while(gpsdistance > switchDist);   ///<switchDist in jakelibrary (10.0m)

    ///<---------------------------------------- GPS WAYPOINT ------------------------------------------>

    ROS_INFO("Switching to GPS waypoint nav");

    float relVelLanding[3];
    float relPosLanding[3];
    float descentVelocity = -0.4;
    float descentDistance = 0.15;

    float LandAlt = 0.3;
    altitude = drone.Data.altitude.bottom_clearance;

    ROS_INFO("Track Ambulance");

    while(gpsdistance > descentDistance)
    {
        float yaw_angle = drone.Data.CalculateYawAngleToTarget();


        relPos[0] = drone.Data.target_position_relative.point.y;
        relPos[1] = drone.Data.target_position_relative.point.x;
        relPos[2] = 0;
        gpsdistance = norm(relPos);
        // drone.Commands.move_Position_Global(drone.Data.gps_raw.latitude, drone.Data.gps_raw.longitude, drone.Data.gps_raw.altitude + 5.0f, yaw_angle);
        drone.Commands.move_Position_Local(drone.Data.target_position_relative.point.x, drone.Data.target_position_relative.point.y, drone.Data.target_position_relative.point.z + 5.0, yaw_angle, "LOCAL_OFFSET", 1);
        ROS_INFO("Distances: E: %f, N: %f", drone.Data.target_position_relative.point.x, drone.Data.target_position_relative.point.y);
        ros::spinOnce();
        rate.sleep();
    }


    // Land and disarm
    ROS_INFO("Landing and disarming");
    drone.Commands.request_LandingAuto();

    // Exit
    return 0;
}