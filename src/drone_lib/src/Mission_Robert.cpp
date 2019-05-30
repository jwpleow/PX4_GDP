#include "headers/gdpdrone.h"

bool detectObstacle(float right, float centre, float left, float range);
float distanceToObstacle(float right, float centre, float left, float range, float angle_max, float angle_min);
float orientationToObstacle(float right, float centre, float left, float range, float angle_max, float angle_min);
float heightOfObstacle(float right, float centre, float left, float range, float z);
float qtToEuler(float x, float y, float z, float w, std::vector<float> vector1);

struct Avoidance
{
    float horizontal, vertical;
    float orientation;
};

struct gazeboposition
{
    float x, y, z; 
};

Avoidance avoidance;
gazeboposition gazeboPosition;

int main(int argc, char **argv)
{
    // Initialise node
    ros::init(argc, argv, "robert_node");

    // Create drone object, this sets everything up
    GDPdrone drone;

    // Set the rate. Default working frequency is 25 Hz
    float loop_rate = 10.0;
    ros::Rate rate = ros::Rate(loop_rate);

    // Initialise and Arm
    drone.Commands.await_Connection();
    drone.Commands.set_Offboard();
    drone.Commands.set_Armed();

    // MISSION STARTS HERE:
    // Request takeoff at 1m altitude. At 25Hz = 10 seconds
    float altitude = 0.50;
    int time_takeoff = 100;

    // Initialize position 
    gazeboPosition.x = 0; 
    gazeboPosition.z = altitude; 

    drone.Commands.request_Takeoff(altitude, time_takeoff);

    ros::Duration(5.0).sleep(); // sleep for 5 seconds

    while (drone.Data.local_pose.pose.position.x < 20){

    // if (detectObstacle(drone.Data.lidar.ranges[0], drone.Data.lidar.ranges[1], drone.Data.lidar.ranges[2], 11.0))
    // {
    //     //avoidance.horizontal = distanceToObstacle(drone.Data.lidar.ranges[0], drone.Data.lidar.ranges[1], drone.Data.lidar.ranges[2], drone.Data.lidar.range_max, drone.Data.lidar.angle_max, drone.Data.lidar.angle_min);
    //     //avoidance.orientation = orientationToObstacle(drone.Data.lidar.ranges[0], drone.Data.lidar.ranges[1], drone.Data.lidar.ranges[2], drone.Data.lidar.range_max, drone.Data.lidar.angle_max, drone.Data.lidar.angle_min);
    //     ROS_INFO("THERE IS AN OBSTACLE @ [%f]: ", distanceToObstacle(drone.Data.lidar.ranges[0], drone.Data.lidar.ranges[1], drone.Data.lidar.ranges[2], drone.Data.lidar.range_max, drone.Data.lidar.angle_max, drone.Data.lidar.angle_min));
            
    //         // fly up till obstacle not detected
    //         while(detectObstacle(drone.Data.lidar.ranges[0], drone.Data.lidar.ranges[1], drone.Data.lidar.ranges[2], 11.0))
    //         {
    //             drone.Commands.move_Position_Local(0.0, 0.0, 0.3, 0, "BODY_OFFSET");
    //             ROS_INFO("Moving up.");
    //             ros::spinOnce();
    //             rate.sleep();
    //         }
    //         // move an extra metre up afterwards
            
    //         drone.Commands.move_Position_Local(0.0, 0.0, 1, 0, "BODY_OFFSET");
    //         ros::spinOnce();
    //         ros::Duration(1.5).sleep();
    // }
    // else
    // {
    //     drone.Commands.move_Position_Local(0.4, 0, 0, 0, "BODY_OFFSET");
    //     ros::spinOnce();
    //     rate.sleep();
    // }


    if (detectObstacle(drone.Data.lidar.ranges[0], drone.Data.lidar.ranges[1], drone.Data.lidar.ranges[2], 11.0))
    {
        //avoidance.horizontal = distanceToObstacle(drone.Data.lidar.ranges[0], drone.Data.lidar.ranges[1], drone.Data.lidar.ranges[2], drone.Data.lidar.range_max, drone.Data.lidar.angle_max, drone.Data.lidar.angle_min);
        //avoidance.orientation = orientationToObstacle(drone.Data.lidar.ranges[0], drone.Data.lidar.ranges[1], drone.Data.lidar.ranges[2], drone.Data.lidar.range_max, drone.Data.lidar.angle_max, drone.Data.lidar.angle_min);
        ROS_INFO("THERE IS AN OBSTACLE @ [%f]: ", distanceToObstacle(drone.Data.lidar.ranges[0], drone.Data.lidar.ranges[1], drone.Data.lidar.ranges[2], drone.Data.lidar.range_max, drone.Data.lidar.angle_max, drone.Data.lidar.angle_min));
            
            // fly up till obstacle not detected
            while(detectObstacle(drone.Data.lidar.ranges[0], drone.Data.lidar.ranges[1], drone.Data.lidar.ranges[2], 11.0))
            {
                drone.Commands.move_Velocity_Local(0.0, 0.0, 0.5, 0, "BODY_OFFSET");
                ROS_INFO("Moving up.");
                ros::spinOnce();
                rate.sleep();
            }
            
            // move an extra bit up afterwards
            for (int i = 0; i < 20; i++){
                drone.Commands.move_Velocity_Local(0.0, 0.0, 0.5, 0, "BODY_OFFSET");
                ros::spinOnce();
                rate.sleep();
            }
    }
    else
    {
        ROS_INFO("Moving Forward");
        drone.Commands.move_Velocity_Local(0.0, 0.5, 0, 0, "BODY_OFFSET");
        ros::spinOnce();
        rate.sleep();
    }

    


    // ROS_INFO("da [%f]", avoidance.horizontal);
    // // Go in the direction of the obstacle and hover over it, but store the current height 
    // while(drone.Data.local_pose.pose.position.x < avoidance.horizontal + 0.075){
    //     drone.Commands.move_Position_Local(0.5, 0, 0, 0, "BODY_OFFSET");  
    //     ROS_INFO("Moving x1");
    //     ros::spinOnce();
    //     rate.sleep();
    // }

    // while(0.8 * drone.Data.local_pose.pose.position.z > drone.Data.infrared_altitude.bottom_clearance){
    //     drone.Commands.move_Position_Local(0.5, 0, 0, 0, "BODY_OFFSET");
    //     ROS_INFO("Moving x2");
    //     ros::spinOnce();
    //     rate.sleep();
    // }

}

    // Land and disarm
    drone.Commands.request_LandingAuto();

    return 0;
}

bool detectObstacle(float right, float centre, float left, float range)
{
    bool isObstacle;
    if ((right < range) || (centre < range) || (left < range))
        isObstacle = 1;
    else
        isObstacle = 0;
    return isObstacle;
}

float distanceToObstacle(float right, float centre, float left, float range, float angle_max, float angle_min)
{
    float distance;
    if (isfinite(right) == 1 && isfinite(centre) == 1 && isfinite(left) == 1)
        distance = std::max(std::max(right, centre), left);
    else if (isfinite(right) == 0)
        distance = std::max(centre, left);
    else if (isfinite(left) == 0)
        distance = std::max(centre, right);
    else if (isfinite(centre) == 0)
        distance = std::max(left, right);
    else if (isfinite(right) == 0 && isfinite(centre) == 0)
        distance = left;
    else if (isfinite(right) == 0 && isfinite(left) == 0)
        distance = centre;
    else if (isfinite(centre) == 0 && isfinite(left) == 0)
        distance = left;
    else if (isfinite(right) == 0 && isfinite(centre) == 0 && isfinite(left) == 0)
        distance = range * 2;
    else ROS_INFO("Distance to obstacle calculation error");
    return distance;
}

float orientationToObstacle(float right, float centre, float left, float range, float angle_max, float angle_min)
{
    float distance, angleOr;
    if (isfinite(right) == 1 && isfinite(centre) == 1 && isfinite(left) == 1)
        distance = std::max(std::max(right, centre), left);
    else if (isfinite(right) == 0)
        distance = std::max(centre, left);
    else if (isfinite(left) == 0)
        distance = std::max(centre, right);
    else if (isfinite(centre) == 0)
        distance = std::max(left, right);
    else if (isfinite(right) == 0 && isfinite(centre) == 0)
        distance = left;
    else if (isfinite(right) == 0 && isfinite(left) == 0)
        distance = centre;
    else if (isfinite(centre) == 0 && isfinite(left) == 0)
        distance = left;
    else if (isfinite(right) == 0 && isfinite(centre) == 0 && isfinite(left) == 0)
        distance = range * 2;

    if (distance == right)
        angleOr = angle_max;
    else if (distance == left)
        angleOr = angle_min;
    else if (distance == centre)
        angleOr = 0;

    return angleOr;
}

float heightOfOBstacle(float right, float centre, float left, float range, float z)
{
    while (right < range && centre < range && left < range)
    {
        z = z + 0.1;
        return z;
    }
}

/*
void qtToEuler(float x, float y, float z, float w, std::vector<float> vector1){ 
    float sinr_cosp = +2.0 * (w * x * y * z);
    float cosr_cosp = +1.0 - 2.0 * (x * x + y + y);
    float roll = atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    float pitch;
    float sinp = +2.0 * (w * y - z * x);
    if (fabs(sinp) >= 1)
        pitch = copysign(M_PI / 2, sinp);
    else
        pitch = asin(sinp);

    // yaw (z-axis rotation)
    float siny_cosp = +2.0 * (w * z + x * y);
    float cosy_cosp = +1.0 - 2.0 * (y * y + z * z);
    float yaw = atan2(siny_cosp, cosy_cosp);

    vector1.push_back(roll);
    vector1.push_back(pitch);
    vector1.push_back(yaw);
}
*/