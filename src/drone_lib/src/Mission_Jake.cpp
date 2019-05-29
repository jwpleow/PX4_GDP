#include "headers/gdpdrone.h"
#include "headers/jakelibrary.h"

int main(int argc, char **argv)
{
    // Initialise node
    ros::init(argc, argv, "jake_node");

    // Create drone object, this sets everything up
    GDPdrone drone;

    // Set the rate. Default working frequency is 25 Hz
    float loop_rate = 10;
    ros::Rate rate = ros::Rate(loop_rate);

    // Initialise and Arm
    drone.Commands.await_Connection();
    drone.Commands.set_Offboard();
    drone.Commands.set_Armed();

    // MISSION STARTS HERE:
    // Request takeoff at 5.77m altitude. 
    float altitude = 5.77;
    int time_takeoff = 50; // 5 seconds at 10 Hz
    ROS_INFO("Setting altitude to 5.77 m");
    drone.Commands.request_Takeoff(altitude, time_takeoff);

    // Initialising
    double switchDist = 5.0; // In reality, camera should start working when it is 8m away. However, GPS is on car NOT THE PLATFORM so probably 6.5 m max
    double droneVel[3];
    double droneAcc[3];
   	double relPos[3];
    double distance;
   	distance = norm(relPos);
   	droneInitVel(dronePos, targPos, droneVel);

   	// Command 1, set drone velocity to the calculated initial velocity in 1 second.
   	ROS_INFO("Initialising drone velocity")
    // Change this to a while loop comparing measured drone velocity and commanded drone velocity
   	for (int count = 1; count < 10; count++) {
   		drone.Commands.move_Velocity_Local(droneVel[1], droneVel[0], -droneVel[2], 0, "LOCAL_OFFSET");
   		ros::spinOnce();
   		rate.sleep;
   	}

   	// Actual proportional navigation algorithm
    ROS_INFO("Starting proportional navigation algorithm");
    while (relPos > switchDist) {
        droneAccComp(relPos, relVel, droneAcc);
        drone.Commands.move_Acceleration_Local();
    }

    // Land and disarm
    ROS_INFO("Landing and disarming")
    drone.Commands.request_LandingAuto();

    // Exit
    return 0;
}