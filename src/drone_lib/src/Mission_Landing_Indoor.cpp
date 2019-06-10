#include "headers/gdpdrone.h"
#include "headers/jakelibrary.h"

int main(int argc, char **argv)
{
    // Initialise node
    ros::init(argc, argv, "landing_indoor_node");

    // Create drone object, this sets everything up
    GDPdrone drone;

    // Save data to flight_data
    drone.Data.start_rosbag();

    //Set the rate. Default working frequency is 25Hz.
    float loop_rate = 10.0f;
    ros::Rate rate = ros::Rate(loop_rate);

    // Initialise and Arm
    drone.Commands.await_Connection();
    drone.Commands.set_Offboard();
    drone.Commands.set_Armed();

    // MISSION STARTS HERE:
    //Request takeoff at 5.77m altitude.
    float takeoff_altitude = 0.4f;
    float time_takeoff = 80;
    ROS_INFO("Setting altitiude to 0.4 m.");
    drone.Commands.request_Takeoff(takeoff_altitude, time_takeoff);

    ROS_INFO("Stabilising over target...");


    float relVelLanding[3];
    float relPosLanding[3];
    float descentVelocity = -0.15f;
    float closeDistance = 5.0f;
    float descentDistance = 0.1f;
    float camdistance = 5.0f;
    float LandAlt = 0.1;
    double altitude = drone.Data.altitude.bottom_clearance;

    gpsdistance = 5.0f;

    ///< while not detecting ARtag OR at too high of an altitude OR too far according to camera
    while(camdistance > descentDistance || LandAlt < altitude)
    {

        altitude = drone.Data.altitude.bottom_clearance;
        ROS_INFO("Altitude is: %f", altitude);

        ///< Calculate camdistance if detected
        if (drone.Data.vishnu_cam_detection.data)
        {
            relPosLanding[0] = drone.Data.vishnu_cam_data.linear.x;
            relPosLanding[1] = drone.Data.vishnu_cam_data.linear.y;
            relPosLanding[2] = 0.0;
            camdistance = norm(relPosLanding);
        }



        if (camdistance > descentDistance && drone.Data.vishnu_cam_detection.data)   ///< - close enough and target seen, use vishnu cam data to centre over target
        {
            ROS_INFO("tag seen - using vishnu data to centre over target");

            //pause for a sec so that drone has 0 pitch
            for(int count = 1; count < 11 ; count++)
            {
                drone.Commands.move_Velocity_Local(0.0f, 0.0f, 0.0f, 0.0f, "BODY_OFFSET");
                ros::spinOnce();
                rate.sleep();
            }

            ///<check if still seeing cam after righting
            if (drone.Data.vishnu_cam_detection.data)
            {

                ///<--------------------- vishnu test------------------->

                // // Update position using Vishnu's cam - vishnu's cam gives body frame (xyz) = (right down up),

                relPosLanding[0] = drone.Data.vishnu_cam_data.linear.x; ///< move right
                relPosLanding[1] = drone.Data.vishnu_cam_data.linear.y; ///< move backward
                relPosLanding[2] = 0.0;
                velCamPosMap(relPosLanding, relVelLanding); // Calculate velocities needed to get towards target in body frame


                ROS_INFO("Distance from target according to cam: %f, move forward: %f, move right: %f", camdistance, -drone.Data.vishnu_cam_data.linear.y, drone.Data.vishnu_cam_data.linear.x);
                for(int count = 1; count < camdistance / velocitynorm * loop_rate ; count++)
                {
                    drone.Commands.move_Velocity_Local(relVelLanding[0], -relVelLanding[1], 0.0f, 0.0f, "BODY_OFFSET");
                    ros::spinOnce();
                    rate.sleep();
                }
            }

        }
        else if (camdistance < descentDistance && drone.Data.vishnu_cam_detection.data) ///< very close - start descending
        {

            ROS_INFO("Close enough, start descending");
            //pause for a sec so that drone has 0 pitch
            for(int count = 1; count < 11 ; count++)
            {
                drone.Commands.move_Velocity_Local(0.0f, 0.0f, 0.0f, 0.0f, "BODY_OFFSET");
                ros::spinOnce();
                rate.sleep();
            }

            ///<check if still seeing cam after righting
            if (drone.Data.vishnu_cam_detection.data)
            {


                // // Update position using Vishnu's cam - vishnu's cam gives body frame (xyz) = (right down up),

                relPosLanding[0] = drone.Data.vishnu_cam_data.linear.x; ///< move right
                relPosLanding[1] = drone.Data.vishnu_cam_data.linear.y; ///< move backward
                relPosLanding[2] = 0.0;
                velCamPosMap(relPosLanding, relVelLanding); // Calculate velocities needed to get towards target in body frame - calculates camdistance and velocitynorm too


                ROS_INFO("Distance from target according to cam: %f, move forward: %f, move right: %f", camdistance, -drone.Data.vishnu_cam_data.linear.y, drone.Data.vishnu_cam_data.linear.x);
                for(int count = 1; count < camdistance / velocitynorm * loop_rate ; count++)
                {
                    drone.Commands.move_Velocity_Local(relVelLanding[0], -relVelLanding[1], -0.1f, 0.0f, "BODY_OFFSET");
                    ros::spinOnce();
                    rate.sleep();
                }
            }

        }
        else   ///< else, rotate around to try to find target (circle + slight descent)
        {
            ROS_INFO("No target detected, rotating & descending to search");
            drone.Commands.move_Velocity_Local(0.0f, 0.0f, -0.05f, 30.0f, "LOCAL_OFFSET");
            ros::spinOnce();
            rate.sleep();
        }
    }

    // Land and disarm
    ROS_INFO("Landing Now");
    drone.Commands.request_LandingAuto();


    return 0;
}