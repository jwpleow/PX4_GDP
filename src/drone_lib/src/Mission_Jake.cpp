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
    float altitude = 5.77;
    float setAltitude = 5.77f;
    int time_takeoff = 80; // 5 seconds at 10 Hz
    ROS_INFO("Setting altitude to 5.77 m");
    drone.Commands.request_Takeoff(altitude, time_takeoff);

    InitialiseJakeCode(drone.Data.target_position_relative.point.x, drone.Data.target_position_relative.point.y, drone.Data.target_position_relative.point.z);

   	// Command 1, set drone velocity to the calculated initial velocity in 1 second.
   	ROS_INFO("Initialising drone velocity");
    // Change this to a while loop comparing measured drone velocity and commanded drone velocity
   	
    drone.Commands.Initialise_Velocity_for_AccelCommands(droneVel[1], droneVel[0], -droneVel[2]);
    
    float initial_yaw = atan2(droneVel[0], droneVel[1]) * 180.0 / 3.14159;
    // turn drone to point in that direction
    ROS_INFO("Turning to direction of inital velocity: %f degrees", initial_yaw); // assume 10 degrees / second?
    for (int count = 1; count < floor(initial_yaw / 10 * loop_rate); count++){
        drone.Commands.move_Position_Local(0.0f, 0.0f, 0.0f, initial_yaw , "LOCAL_OFFSET", count);
        ros::spinOnce();
        rate.sleep();
    }




   	// Actual proportional navigation algorithm
    ROS_INFO("Starting proportional navigation algorithm"); 

    do {
        ROS_INFO("%f");
        droneAccComp(relPos, relVel, droneAcc);
        accFix = altitudeFix(drone.Data.target_position_relative.point.z, setAltitude);
        ROS_INFO("Accelerations needed: x: %f, y: %f, z: %f", droneAcc[1], droneAcc[0], droneAcc[2]);
        drone.Commands.move_Acceleration_Local_Trick(droneAcc[1],droneAcc[0], accFix, "LOCAL_OFFSET", loop_rate);

        for (int i = 0; i < 3; ++i) {
          relPosOld[i] = relPos[i];
        }

        relPos[0] = drone.Data.target_position_relative.point.y;
        relPos[1] = drone.Data.target_position_relative.point.x;
        relPos[2] = drone.Data.target_position_relative.point.z;

        gpsdistance = norm(relPos);

        ROS_INFO("Distance to target: %f", gpsdistance);

        velFromGPS(relPos, relPosOld, loop_rate, relVel);


        ros::spinOnce();
        rate.sleep();



    } while(gpsdistance > switchDist); ///<switchDist in jakelibrary (5.0m)

    ///<---------------------------------------- GPS WAYPOINT ------------------------------------------>

    ROS_INFO("Switching to GPS waypoint nav");
 
    float relVelLanding[3];
    float relPosLanding[3];
    float descentVelocity = -0.3;
    float descentDistance = 0.15;

    float LandAlt = 0.3;
    altitude = drone.Data.altitude.bottom_clearance;

    do{

        altitude = drone.Data.altitude.bottom_clearance;
        ROS_INFO("Altitude is: %f", altitude);

        ///< If vishnu ARtag not detected - use GPS to move towards target
        if (gpsdistance > descentDistance)
        {

            // Update position using GPS
            relPosLanding[0] = drone.Data.target_position_relative.point.x; ///< east offset
            relPosLanding[1] = drone.Data.target_position_relative.point.y; ///< north offset
            relPosLanding[2] = 0.0;

            velPosMap(relPosLanding, relVelLanding); // Calculate velocity needed - output flips east/north???
            // Do I want to yaw to face the front?
            ROS_INFO("Moving towards target via GPS to target position, distance: %f, x: %f, y: %f", gpsdistance, relPosLanding[0], relPosLanding[1]);
            drone.Commands.move_Velocity_Local(relVelLanding[0], relVelLanding[1], 0.0, 0.0, "LOCAL_OFFSET");
            ros::spinOnce();
            rate.sleep();
        }
        else   ///< - close enough , descend
        {
            ROS_INFO("At descent distance - descending");


            relPosLanding[0] = drone.Data.target_position_relative.point.x;
            relPosLanding[1] = drone.Data.target_position_relative.point.y;
            relPosLanding[2] = 0.0;
            velPosMap(relPosLanding, relVelLanding);
            // move velocity is (xyz) = (right forward up)
            drone.Commands.move_Velocity_Local(relVelLanding[0], relVelLanding[1], descentVelocity, 0.0, "LOCAL_OFFSET");
            ros::spinOnce();
            rate.sleep();


        }
    }while(gpsdistance > descentDistance || LandAlt < altitude);


    // Land and disarm
    ROS_INFO("Landing and disarming");
    drone.Commands.request_LandingAuto();

    // Exit
    return 0;
}