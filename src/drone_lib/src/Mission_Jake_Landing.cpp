#include "headers/gdpdrone.h"
#include "headers/jakelibrary.h"

int main(int argc, char **argv)
{
    // Initialise node
    ros::init(argc, argv, "jake_landing_node");

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
    float takeoff_altitude = 5.77f;
    float time_takeoff = 80;
    ROS_INFO("Setting altitiude to 5.77 m.");
    drone.Commands.request_Takeoff(takeoff_altitude, time_takeoff);

    ROS_INFO("Stabilising over target...");


    float relVelLanding[3];
    float relPosLanding[3];
    float descentVelocity = -0.15f;
    float closeDistance = 5.0f;
    float descentDistance = 0.2f;
    float camdistance = 5.0f;
    float LandAlt = 0.1;
    double altitude = drone.Data.altitude.bottom_clearance;

    gpsdistance = 5.0f;

    ///< while not detecting ARtag OR at too high of an altitude OR too far according to camera
    while(camdistance > descentDistance || LandAlt < altitude || !drone.Data.vishnu_cam_detection.data) 
    {

        altitude = drone.Data.altitude.bottom_clearance;
        ROS_INFO("Altitude is: %f", altitude);

        ///< If vishnu ARtag not detected and far - use GPS to move towards target
        if (gpsdistance > descentDistance && !drone.Data.vishnu_cam_detection.data)
        {

            // Update position using GPS
            relPosLanding[0] = drone.Data.target_position_relative.point.x; ///< east offset
            relPosLanding[1] = drone.Data.target_position_relative.point.y; ///< north offset
            relPosLanding[2] = 0.0;

            velPosMap(relPosLanding, relVelLanding); // Calculate velocity needed - output flips east/north???
            // Do I want to yaw to face the front?
            ROS_INFO("Moving towards target via GPS to target position, distance: %f, x: %f, y: %f", gpsdistance, relPosLanding[0], relPosLanding[1]);
            drone.Commands.move_Velocity_Local(relVelLanding[0], relVelLanding[1], relVelLanding[2], 0.0, "LOCAL_OFFSET");
            ros::spinOnce();
            rate.sleep();
        }
        else if (camdistance > descentDistance && drone.Data.vishnu_cam_detection.data)   ///< - close enough and target seen, use vishnu cam data to centre over target
        {
            ROS_INFO("Switching to vishnu's data to centre");

            //pause for a sec so that drone has 0 pitch
             for(int count = 1; count < 11 ; count++)
            {
                drone.Commands.move_Velocity_Local(0, 0, 0, 0,"LOCAL_OFFSET");
                ros::spinOnce();
                rate.sleep();
            }

            ///<check if still seeing cam after righting
            if (camdistance < descentDistance && drone.Data.vishnu_cam_detection.data){

            ///<--------------------- vishnu test------------------->

            // // Update position using Vishnu's cam - vishnu's cam gives body frame (xyz) = (right down up),

            relPosLanding[0] = drone.Data.vishnu_cam_data.linear.y;
            relPosLanding[1] = drone.Data.vishnu_cam_data.linear.x;
            relPosLanding[2] = 0.0;
            // velPosMap(relPosLanding, relVelLanding); // Calculate velocities needed to get towards target in body frame

            // move velocity is (xyz) = (right forward up)
            // drone.Commands.move_Velocity_Local(relVelLanding[0], relVelLanding[1], descentVelocity, 0.0, "BODY_OFFSET");

            camdistance = norm(relPosLanding);
            ROS_INFO("Distance from target according to cam: %f, move forward: %f, move right: %f", camdistance, -drone.Data.vishnu_cam_data.linear.y, drone.Data.vishnu_cam_data.linear.x);
            for(int count = 1; count < 31 ; count++)
            {
                drone.Commands.move_Position_Local(drone.Data.vishnu_cam_data.linear.x, -drone.Data.vishnu_cam_data.linear.y, 0, 0, "BODY_OFFSET", count);
                ros::spinOnce();
                rate.sleep();
            }
        }

        }
        else if (camdistance < descentDistance && drone.Data.vishnu_cam_detection.data) ///< very close - start descending
        {
            //pause for a sec so that drone has 0 pitch
             for(int count = 1; count < 11 ; count++)
            {
                drone.Commands.move_Velocity_Local(0, 0, -0.05, 0,"LOCAL_OFFSET");
                ros::spinOnce();
                rate.sleep();
            }
            if (camdistance < descentDistance && drone.Data.vishnu_cam_detection.data){

            relPosLanding[0] = drone.Data.vishnu_cam_data.linear.y;
            relPosLanding[1] = drone.Data.vishnu_cam_data.linear.x;
            relPosLanding[2] = 0.0;
            camdistance = norm(relPosLanding);
            ROS_INFO("Very close, Start Descending. Distance from target according to cam: %f", camdistance);
             for(int count = 1; count < 31 ; count++)
            {
                 drone.Commands.move_Position_Local(drone.Data.vishnu_cam_data.linear.x, -drone.Data.vishnu_cam_data.linear.y, -0.5, 0, "BODY_OFFSET", count);
                ros::spinOnce();
                rate.sleep();
            }
        }

        }
        else { ///< else fallback to GPS
            relPosLanding[0] = drone.Data.target_position_relative.point.x; ///< north offset
            relPosLanding[1] = drone.Data.target_position_relative.point.y; ///< east offset
            relPosLanding[2] = 0.0;

            velPosMap(relPosLanding, relVelLanding); // Calculate velocity needed - output flips east/north???
            // Do I want to yaw to face the front?
            ROS_INFO("Moving towards target via GPS to target position, distance: %f, x: %f, y: %f", gpsdistance, relPosLanding[0], relPosLanding[1]);
            drone.Commands.move_Velocity_Local(relVelLanding[0], relVelLanding[1], relVelLanding[2], 0.0, "LOCAL_OFFSET");
            ros::spinOnce();
            rate.sleep();
        }
    }

    // Land and disarm
    ROS_INFO("Landing Now");
    drone.Commands.request_LandingAuto();


    return 0;
}