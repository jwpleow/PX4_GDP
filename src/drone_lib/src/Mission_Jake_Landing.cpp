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
    float descentVelocity = -0.15;
    float descentDistance = 0.15;
    float LandAlt = 0.1;
    double altitude = drone.Data.altitude.bottom_clearance;

    distance = 5.0f;
    ///< while not detecting ARtag or at too high of an altitude
    while(!drone.Data.vishnu_cam_detection.data || LandAlt < altitude) { 

        altitude = drone.Data.altitude.bottom_clearance;
        ROS_INFO("Altitude is: %f", altitude);
      
        ///< If vishnu ARtag not detected - use GPS to move towards target
        if (!drone.Data.vishnu_cam_detection.data){
            ROS_INFO("Moving towards target via PID to target position");
            // Update position using GPS
            relPosLanding[0] = drone.Data.target_position_relative.point.y;
            relPosLanding[1] = drone.Data.target_position_relative.point.x;
            relPosLanding[2] = 0.0;

            
            velPosMap(relPosLanding, relVelLanding); // Calculate velocity needed
            // Do I want to yaw to face the front?
            drone.Commands.move_Velocity_Local(relVelLanding[0], relVelLanding[1], relVelLanding[2], 0.0, "LOCAL_OFFSET");
            ros::spinOnce();
            rate.sleep();
        }
        else { ///< - close enough , use vishnu cam data to land
            ROS_INFO("At descent distance - switching to vishnu's data");

            // Update position using Vishnu's cam - vishnu's cam gives body frame (xyz) = (forward right up),  
            relPosLanding[0] = drone.Data.vishnu_cam_data.linear.y;
            relPosLanding[1] = drone.Data.vishnu_cam_data.linear.x;
            relPosLanding[2] = 0.0;
            velPosMap(relPosLanding, relVelLanding); // Calculate velocities needed to get towards target in body frame

            // move velocity is (xyz) = (right forward up)
            drone.Commands.move_Velocity_Local(relVelLanding[0], relVelLanding[1], descentVelocity, 0.0, "BODY_OFFSET");

            ros::spinOnce();
            rate.sleep();
        }
    } 

     // Land and disarm
    ROS_INFO("Landing Now");
    drone.Commands.request_LandingAuto();


    return 0;
}