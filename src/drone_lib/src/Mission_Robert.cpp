#include "headers/gdpdrone.h"
#include <fstream>
#include <string.h>
#include <iterator>
#include <random>
#include <cmath>

using namespace std;

bool detectObstacle(float right, float centre, float left, float range);
float distanceToObstacle(float right, float centre, float left, float range);
float orientationToObstacle(float right, float centre, float left, float range, float angle_max, float angle_min);
float heightOfObstacle(float right, float centre, float left, float range, float z);
void write_to_file(float a);
bool detectIfOverObstacle(float a, float b);
void qtToEuler(float x, float y, float z, float w, float angles[3]);

struct Avoidance
{
    float horizontal;
    float vertical;
    float orientation;
};

struct gazeboposition
{
    float x, y, z;
};

struct currentobstacle
{
    float left, centre, right;
    std::vector<float> x_pos = {0};
    std::vector<float> y_pos = {0};
};

struct finalposition
{
    float x, y, z;
};

float Orientation[3];

Avoidance avoidance;
gazeboposition gazeboPosition;
currentobstacle currentObstacle;
finalposition finalPosition;

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
    int time_takeoff = 50;
    drone.Commands.request_Takeoff(altitude, time_takeoff);

    // Store altitude after takeoff
    gazeboPosition.z = altitude;
    gazeboPosition.x = 0;

    // Set final position
    finalPosition.x = 10;
    finalPosition.y = 10;
    finalPosition.z = 0;

    // Check obstacle
    bool FlSalam = 0;

    // Track the obstacle
    float rotation;
    std::vector<float> goalOrientation = {0};

    // Starting mission profile
    while (drone.Data.local_pose.pose.position.x < 20)
    {
        // Assume there is no obstacle in sight
        FlSalam = 0;

        // Orientation of goal with respect to current position; modify heading angle accordingly
        /*rotation = atan2(finalPosition.x - drone.Data.local_pose.pose.position.x, finalPosition.y - drone.Data.local_pose.pose.position.y);
        rotation = rotation * 180 / M_PI;
        ROS_INFO("Rotation is [%f]", rotation);
        while(drone.Data.compass_heading.data < rotation)
        {
            drone.Commands.move_Position_Local(0, 0, 0, 5, "BODY_OFFSET");
             ros::spinOnce();
             rate.sleep();
        }*/

        // Calculate the relative angle at which the object

        if (detectObstacle(drone.Data.lidar.ranges[0], drone.Data.lidar.ranges[1], drone.Data.lidar.ranges[2], drone.Data.lidar.range_max) == 1)
        {
            avoidance.horizontal = distanceToObstacle(drone.Data.lidar.ranges[0], drone.Data.lidar.ranges[1], drone.Data.lidar.ranges[2], drone.Data.lidar.range_max);
            ROS_INFO("Obstacle is at: [%f]", avoidance.horizontal + drone.Data.local_pose.pose.position.x);
            avoidance.orientation = orientationToObstacle(drone.Data.lidar.ranges[0], drone.Data.lidar.ranges[1], drone.Data.lidar.ranges[2], drone.Data.lidar.range_max, drone.Data.lidar.angle_max, drone.Data.lidar.angle_min);

            // Store the initial distance to the obstacle
            currentObstacle.right = drone.Data.lidar.ranges[0];
            currentObstacle.centre = drone.Data.lidar.ranges[1];
            currentObstacle.left = drone.Data.lidar.ranges[2];

            // Store the x-position of the obstacle
            currentObstacle.x_pos.push_back(avoidance.horizontal + drone.Data.local_pose.pose.position.x);
            FlSalam = 1;

            // Move between the two obstacles
            while (drone.Data.local_pose.pose.position.x < (currentObstacle.x_pos[currentObstacle.x_pos.size() - 2] + currentObstacle.x_pos[currentObstacle.x_pos.size() - 1]) / 2)
            {
                if (detectIfOverObstacle(drone.Data.altitude.bottom_clearance, drone.Data.local_pose.pose.position.z) == 0)
                {
                    // Not over obstacle, can go up in height
                    drone.Commands.move_Velocity_Local(0, 0.5, 0.25, 0, "BODY_OFFSET");
                }
                else
                {
                    // Over obstacle, do not go up in height
                    drone.Commands.move_Velocity_Local(0, 0.5, 0, 0, "BODY_OFFSET");
                }
                ros::spinOnce();
                rate.sleep();
            }
        }

        else
        {
            drone.Commands.move_Velocity_Local(0, 0.5, 0, 0, "BODY_OFFSET");
            //gazeboPosition.x = gazeboPosition.x + 0.5;
            currentObstacle.right = drone.Data.lidar.range_max;
            currentObstacle.centre = drone.Data.lidar.range_max;
            currentObstacle.left = drone.Data.lidar.range_max;
            ROS_INFO("There is no obstacle!");
            qtToEuler(drone.Data.imu.orientation.x, drone.Data.imu.orientation.y, drone.Data.imu.orientation.z, drone.Data.imu.orientation.w, Orientation);
        }

        // Reach a height where the sensors do not detect any obstacle
        while (drone.Data.lidar.ranges[0] < drone.Data.lidar.range_max && drone.Data.lidar.ranges[1] < drone.Data.lidar.range_max && drone.Data.lidar.ranges[2] < drone.Data.lidar.range_max)
        {
            // Move up until sensors do not detect anything
            drone.Commands.move_Velocity_Local(0, 0, 0.5, 0, "BODY_OFFSET");

            // Now check if there is a new obstacle behind the current one and if so, stop going up;
            if ((currentObstacle.right < 0.8 * drone.Data.lidar.ranges[0]) &&
                currentObstacle.centre < 0.8 * drone.Data.lidar.ranges[1] &&
                currentObstacle.left < 0.8 * drone.Data.lidar.ranges[2])
            {
                break;
            }

            // Store the height of the obstacle
            currentObstacle.y_pos.push_back(drone.Data.local_pose.pose.position.z);
            ros::spinOnce();
            rate.sleep();
        }

        // Safety margin to account for noise/bad sensor readings and pitching angle
        while (drone.Data.local_pose.pose.position.z < currentObstacle.y_pos[currentObstacle.y_pos.size() - 1] + 0.45)
        {
            // Move up until sensors do not detect anything
            drone.Commands.move_Velocity_Local(0, 0, 0.5, 0, "BODY_OFFSET");
            ros::spinOnce();
            rate.sleep();
        }

        // Go in the direction of the obstacle and hover over it, but store the current height
        while (drone.Data.local_pose.pose.position.x < currentObstacle.x_pos[currentObstacle.x_pos.size() - 1] + 0.25)
        {
            // Use pitch to input a z velocity such that the drone will not decrease in height
            qtToEuler(drone.Data.imu.orientation.x, drone.Data.imu.orientation.y, drone.Data.imu.orientation.z, drone.Data.imu.orientation.z, Orientation);
            avoidance.orientation = avoidance.orientation + Orientation[3] / 180 * M_PI;

            // Go forward command
            drone.Commands.move_Velocity_Local(0, 1.5, 0, 0, "BODY_OFFSET");
            ros::spinOnce();
            rate.sleep();
        }

        // Move forward
        while (0.8 * gazeboPosition.z > drone.Data.altitude.bottom_clearance && FlSalam == 1)
        {
            drone.Commands.move_Velocity_Local(0, 0.5, 0, 0, "BODY_OFFSET");
            qtToEuler(drone.Data.imu.orientation.x, drone.Data.imu.orientation.y, drone.Data.imu.orientation.z, drone.Data.imu.orientation.w, Orientation);
            ros::spinOnce();
            rate.sleep();
        }

        // finalPosition.x ++;
        // finalPosition.y ++;

        ros::spinOnce();
        rate.sleep();
    }

    // Land
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

float distanceToObstacle(float right, float centre, float left, float range)
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

bool detectIfOverObstacle(float a, float b)
{
    if (a < 0.5 * b)
        return 1;
    else
        return 0;
}

void write_to_file(float a)
{
    std::ofstream myfile("data.txt", std::ofstream::out | std::ofstream::app);
    myfile << a << std::endl;
    myfile.flush();
}

void qtToEuler(float x, float y, float z, float w, float angles[3])
{
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

    angles[0] = roll * 180 / M_PI;
    angles[1] = pitch * 180 / M_PI;
    angles[2] = yaw * 180 / M_PI;
}

void addGaussianNoise(std::vector<float> SensorRanges, const double mean, const double stddev){
    std::default_random_engine generator;
    std::normal_distribution<double> dist(mean, stddev);

    for (auto& x : SensorRanges)
        x = x + dist(generator);
}

