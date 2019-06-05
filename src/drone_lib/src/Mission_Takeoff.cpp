#include "headers/gdpdrone.h"

int main(int argc, char **argv)
{
    // Initialise node
    ros::init(argc, argv, "takeoff_node");

    // Create drone object, this sets everything up
    GDPdrone drone;

    // Save data to flight_data
    // drone.Data.start_rosbag();

    // Set the rate. Default working frequency is 25 Hz
    float loop_rate = 25.0;
    ros::NodeHandle nh;
    ros::Rate rate = ros::Rate(loop_rate);

    // Initialise and Arm
    drone.Commands.await_Connection();
    drone.Commands.set_Offboard();
    drone.Commands.set_Armed();

    // Take-off and hover
    float altitude = 2;
    int time_takeoff = 150;
    drone.Commands.request_Takeoff(altitude, time_takeoff);

    // Land and disarm
    ROS_INFO("Landing Now");
    drone.Commands.request_LandingAuto();
}
