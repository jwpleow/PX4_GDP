#include "headers/gdpdrone.h"
#include "headers/jakelibrary.h"

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
    int time_takeoff = 80; // 5 seconds at 10 Hz
    ROS_INFO("Setting altitude to 5.77 m");
    drone.Commands.request_Takeoff(altitude, time_takeoff);

    InitialiseJakeCode(drone.Data.target_position_relative.point.x, drone.Data.target_position_relative.point.y, drone.Data.target_position_relative.point.z);
//
   	// Command 1, set drone velocity to the calculated initial velocity in 1 second.
   	ROS_INFO("Initialising drone velocity");   	
    drone.Commands.Initialise_Velocity_for_AccelCommands(droneVel[1], droneVel[0], -droneVel[2]);

           /// position data is NEU
        Pos[0] = drone.Data.target_position.point.x;
        Pos[1] = drone.Data.target_position.point.y;
        Pos[2] = drone.Data.target_position.point.z;


   	// Actual proportional navigation algorithm
    ROS_INFO("Starting proportional navigation algorithm");
    do {
        droneAccComp(relPos, relVel, droneAcc);
        ROS_INFO("Accelerations needed: x: %f, y: %f, z: %f", droneAcc[1], droneAcc[0], -droneAcc[2]);
        drone.Commands.move_Acceleration_Local_Trick(droneAcc[1], droneAcc[0], -droneAcc[2], "LOCAL_OFFSET", loop_rate);


        
        for (int i = 0; i < 3; ++i) {
          PosOld[i] = Pos[i];
        }

        relPos[0] = drone.Data.target_position_relative.point.y;
        relPos[1] = drone.Data.target_position_relative.point.x;
        relPos[2] = 0.0;

        distance = norm(relPos);

        ROS_INFO("Distance to target: %f", distance);
        ROS_INFO("Position: x: %f, y: %f, z: %f", drone.Data.target_position_relative.point.y, drone.Data.target_position_relative.point.x, drone.Data.target_position_relative.point.z);
        //ROS_INFO("Velocity: E: %f, N: %f, U: %f", drone.Data.local_velocity.twist.linear.x, drone.Data.local_velocity.twist.linear.y, drone.Data.local_velocity.twist.linear.z);
        ROS_INFO("RelVelocity: E: %f, N: %f, U: %f", relVel[0], relVel[1], relVel[2]);



               /// position data is NEU
        Pos[0] = drone.Data.target_position.point.x;
        Pos[1] = drone.Data.target_position.point.y;
        Pos[2] = drone.Data.target_position.point.z;








        velFromGPS(Pos, PosOld, drone.Data.local_velocity.twist.linear.x, drone.Data.local_velocity.twist.linear.y, drone.Data.local_velocity.twist.linear.z, loop_rate, relVel);




        ros::spinOnce();
        rate.sleep();
    } while(distance > switchDist);

    // Land and disarm
    ROS_INFO("Landing and disarming");
    drone.Commands.request_LandingAuto();

    // Exit
    return 0;
}

