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

Avoidance avoidance;

int main(int argc, char **argv)
{
    // Initialise node
    ros::init(argc, argv, "robert_node");

    // Create drone object, this sets everything up
    GDPdrone drone;

    // Set the rate. Default working frequency is 25 Hz
    float loop_rate = 25.0;
    ros::Rate rate = ros::Rate(loop_rate);

    // Initialise and Arm
    drone.Commands.await_Connection();
    drone.Commands.set_Offboard();
    drone.Commands.set_Armed();

    // MISSION STARTS HERE:
    // Request takeoff at 1m altitude. At 25Hz = 10 seconds
    float altitude = 0.50;
    int time_takeoff = 250;
    drone.Commands.request_Takeoff(altitude, time_takeoff);

    if (detectObstacle(drone.Data.lidar.ranges[0], drone.Data.lidar.ranges[1], drone.Data.lidar.ranges[2], drone.Data.lidar.range_max) == 1)
    {
        avoidance.horizontal = distanceToObstacle(drone.Data.lidar.ranges[0], drone.Data.lidar.ranges[1], drone.Data.lidar.ranges[2], drone.Data.lidar.range_max, drone.Data.lidar.angle_max, drone.Data.lidar.angle_min);
        avoidance.orientation = orientationToObstacle(drone.Data.lidar.ranges[0], drone.Data.lidar.ranges[1], drone.Data.lidar.ranges[2], drone.Data.lidar.range_max, drone.Data.lidar.angle_max, drone.Data.lidar.angle_min);
        ROS_INFO("THERE IS AN OBSTACLE @ [%f]: ", distanceToObstacle(drone.Data.lidar.ranges[0], drone.Data.lidar.ranges[1], drone.Data.lidar.ranges[2], drone.Data.lidar.range_max, drone.Data.lidar.angle_max, drone.Data.lidar.angle_min));
    }

    float z = altitude;
    // Reach a height where the sensors do not detect any obstacle
    while (drone.Data.lidar.ranges[0] < drone.Data.lidar.range_max && drone.Data.lidar.ranges[1] < drone.Data.lidar.range_max && drone.Data.lidar.ranges[2] < drone.Data.lidar.range_max){ 
        drone.Commands.move_Position_Local(0, 0, z, 0, "BODY");
        z = z + 0.05; 
        ros::spinOnce();
        rate.sleep();
    }

    float y = 0;
    ROS_INFO("da [%f]", avoidance.horizontal);
    // Go in the direction of the obstacle and hover over it, but store the current height 
    while(y < avoidance.horizontal * 1.25){
        ROS_INFO("why");
        y += avoidance.horizontal * 1.25;
        drone.Commands.move_Position_Local(y, 0, z, 0, "BODY"); 
        ros::spinOnce();
        rate.sleep();
    }

    // Hover over the obstacle
    int Counter = 0;
    while (Counter < 150){
        drone.Commands.move_Position_Local(y, 0, z, 0, "BODY");
        Counter++;
    }

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