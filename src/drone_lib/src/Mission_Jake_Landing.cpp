#include "headers/gdpdrone.h"
#include "headers/jakelibrary.h"

int main(int argc, char **argv)
{
    // Initialise node
    ros::init(argc, argv, "jake_landing_node");

    // Create drone object, this sets everything up
    GDPdrone drone;

    //Set the rate. Default working frequency is 25Hz.
    float loop_rate = 10.0f;
    ros::Rate rate = ros::Rate(loop_rate);

    // Initialise and Arm
    drone.Commands.await_Connection();
    drone.Commands.set_Offboard();
    drone.Commands.set_Armed();

    // MISSION STARTS HERE:
    //Request takeoff at 5.77m altitude.
    float altitude = 5.77f;
    float time_takeoff = 80;
    ROS_INFO("Setting altitiude to 5.77 m.");
    drone.Commands.request_Takeoff(altitude, time_takeoff);

    ROS_INFO("Stabilising over target...");

    float altitude;
    float relVelLanding[3];
    float relPosLanding[3];
    float descentVelocity = -0.2;
    float descentDistance = 0.05;
    float cutPowerAlt = 0.1;

    do {
        altitude = drone.Data.target_position_relative.point.z;
        relPosLanding[0] = drone.Data.target_position_relative.point.y;
        relPosLanding[1] = drone.Data.target_position_relative.point.x;
        relPosLanding[2] = 0.0;

        velPosMap(relPosLanding, relVelLanding);
        
        // Do I want to yaw to face the front?
        drone.Commands.move_Velocity_Local(relVelLanding[0], relVelLanding[1], relVelLanding[2], 0.0, "LOCAL_OFFSET");

        if (distance < descentDistance) {
            drone.Commands.move_Velocity_Local(0.0, 0.0, descentVelocity, 0.0, "LOCAL_OFFSET");
        }

        else if (altitude < cutPowerAlt) {
            break;
        }

        ros::spinOnce();
        rate.sleep();

    } while (true);

    // Cut power

    drone.Commands.set_Disarmed();

    return 0;
}