#include "headers/gdpdrone.h"

int main(int argc, char **argv)
{
    // Initialise node
    ros::init(argc, argv, "drone_node");

    // Create drone object, this sets everything up
    GDPdrone drone;

    // Save data to flight_data
    drone.Data.start_rosbag();

    // Set the rate. Default working frequency is 25 Hz
    float loop_rate = 25.0;
    ros::NodeHandle nh;
    ros::Rate rate = ros::Rate(loop_rate);

    // Initialise and Arm
    drone.Commands.await_Connection();
    drone.Commands.set_Offboard();

    // Subscribe to RC
    ros::Publisher servo_pub = nh.advertise<mavros_msgs::ActuatorControl>("mavros/actuator_control", 10);

    mavros_msgs::ActuatorControl act_msg;
    ROS_INFO("init done");
    act_msg.group_mix = 1;
    act_msg.controls[5] = 0.7;

    ROS_INFO("set value");

    for (int i = 1; i < 400; i++)
    {
        servo_pub.publish(act_msg);
        ROS_INFO("set one");
        ros::spinOnce();
        rate.sleep();
    }

    for (int i = 1; i < 400; i++)
    {
        act_msg.controls[5] = 0.25;
        servo_pub.publish(act_msg);
        ROS_INFO("set two");
        ros::spinOnce();
        rate.sleep();
    }

    /*
    drone.Commands.set_Armed();

    // MISSION STARTS HERE:
    // Request takeoff at 1m altitude. At 25Hz = 10 seconds
    float altitude = 2;
    int time_takeoff = 100;
    drone.Commands.request_Takeoff(altitude, time_takeoff);

    // Go one meter up and stay there. Total time 10 seconds
    ROS_INFO("Yaw 20 degrees");
    for (int count = 1; count < 100; count++)
    {
        drone.Commands.move_Position_Local(0, 0, 0, 45, "BODY_OFFSET", count);
        ros::spinOnce();
        rate.sleep();
    }
   
    ROS_INFO("Move Left");
    for (int count = 1; count < 100; count++)
    {
        drone.Commands.move_Position_Local(-1, 0, 0, 0, "BODY_OFFSET", count);
        ros::spinOnce();
        rate.sleep();
    }

    ROS_INFO("Move Left Again");
    for (int count = 1; count < 100; count++)
    {
        drone.Commands.move_Position_Local(-1, 0, 0, 0, "BODY_OFFSET", count);
        ros::spinOnce();
        rate.sleep();
    }

    // Go one meter up and stay there. Total time 10 seconds
    ROS_INFO("Move Right");
    for (int count = 1; count < 100; count++)
    {
        drone.Commands.move_Position_Local(2, 0, 0, 0, "BODY_OFFSET", count);
        ros::spinOnce();
        rate.sleep();
    }

    ROS_INFO("Re-Centre");
    for (int count = 1; count < 100; count++)
    {
        drone.Commands.move_Position_Local(-1, 0, 0, 0, "BODY_OFFSET", count);
        ros::spinOnce();
        rate.sleep();
    }

    // Go one meter up and stay there. Total time 10 seconds
    ROS_INFO("Move Backwards");
    for (int count = 1; count < 100; count++)
    {
        drone.Commands.move_Position_Local(0, -1, 0, 0, "BODY_OFFSET", count);
        ros::spinOnce();
        rate.sleep();
    }

    // Go one meter up and stay there. Total time 10 seconds
    ROS_INFO("Move Forwards");
    for (int count = 1; count < 100; count++)
    {
        drone.Commands.move_Position_Local(0, 2, 0, 0, "BODY_OFFSET", count);
        ros::spinOnce();
        rate.sleep();
    }

    ROS_INFO("Re-Centre");
    for (int count = 1; count < 100; count++)
    {
        drone.Commands.move_Position_Local(0, -1, 0, 0, "BODY_OFFSET", count);
        ros::spinOnce();
        rate.sleep();
    }

    /*
    ROS_INFO("Moving to GPS coordinate 1");
    for (int count = 1; count < 300; count++)
    {
        drone.Commands.move_Position_Global(51.412420, -0.641533, 0, 0, "BODY_OFFSET");
        ros::spinOnce();save_data
        rate.sleep();
    }

    ROS_INFO("Moving to GPS coordinate 2");
    for (int count = 1; count < 300; count++)
    {
        drone.Commands.move_Position_Global(51.412576, -0.641681, 0, 0, "BODY_OFFSET");
        ros::spinOnce();
        rate.sleep();
    }

    ROS_INFO("Moving to GPS coordinate 3");
    for (int count = 1; count < 300; count++)
    {
        drone.Commands.move_Position_Global(51.412384, -0.641616, 0, 0, "BODY_OFFSET");
        ros::spinOnce();
        rate.sleep();
    }

    // Land and disarm
    ROS_INFO("Landing Now");
    drone.Commands.request_LandingAuto();
    */

    // Exit
    return 0;
}