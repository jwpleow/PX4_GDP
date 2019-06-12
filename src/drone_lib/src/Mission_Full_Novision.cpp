#include "headers/gdpdrone.h"
#include <fstream>
#include <string.h>
#include <iterator>
#include <random>
#include "headers/jakelibrary.h"
#include <cmath>

void storePosition(struct MissionPositions STEP, float x, float y, float z, float yaw);
bool detectObstacle(float right, float centre, float left, float range);
float degtorad(float a);
float radtodeg(float a);
float distanceToObstacle(float right, float centre, float left, float range);
float orientationToObstacle(float right, float centre, float left, float range, float angle_max, float angle_min);
bool detectIfOverObstacle(float a, float b);
float propControl(float aNow, float aOld, float b);

struct MissionPositions
{
    float x, y, z;
    float yaw;
};

struct positions
{
    MissionPositions InitialCollection;
    MissionPositions FinalCollection;
    MissionPositions InitialObstacleAvoidance;
    MissionPositions FinalObstacleAvoidance;
    MissionPositions InitialTracking;
    MissionPositions FinalTracking;
};

struct Avoidance
{
    float horizontal;
    float vertical;
    float orientation;
};

struct obstacledistance
{
    std::vector<float> left = {0.0};
    std::vector<float> centre = {0.0};
    std::vector<float> right = {0.0};
    std::vector<float> xPosition = {0.0};
    std::vector<float> yPosition = {0.0};
};

positions POSITIONS;
Avoidance avoidance;
obstacledistance ObstacleDistance;

int main(int argc, char **argv)
{
    // Initialise node
    ros::init(argc, argv, "mission_full_novision_node");

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
    float altitude = 0.5;
    int time_takeoff = 50;
    drone.Commands.request_Takeoff(altitude, time_takeoff);


    ///<--------------------------------------- STEP 2  ---- OBSTACLE AVOIDANCE ----------------------------------->

    // Record positions at beggining of mission
    storePosition(POSITIONS.InitialObstacleAvoidance, drone.Data.local_pose.pose.position.x,
                  drone.Data.local_pose.pose.position.y, drone.Data.local_pose.pose.position.z,
                  drone.Data.compass_heading.data);

    // Check obstacle and create variables to record the orientation of obstacles
    bool FlSalam = 0;
    float rotation;
    std::vector<float> goalOrientation = {0};

    // Set MISSION CONDITIONS for Obstacle Avoidance Algorithm

    while (drone.Data.local_pose.pose.position.x < POSITIONS.InitialObstacleAvoidance.x + 10)
    {
        // Assume there is no obstacle at the beggining of each iteration
        FlSalam = 0;

        // Check if there is a visible obstacle
        if (detectObstacle(drone.Data.lidar.ranges[0], drone.Data.lidar.ranges[1], drone.Data.lidar.ranges[2], drone.Data.lidar.range_max) == 1)
        {
            // If yes, then store the distance to the obstacle given by function distanceToObstacle; also record the Orientation of the obstacle to
            // to check if the obstacle is inclined. If inclined, then turn a bit.
            avoidance.horizontal = distanceToObstacle(drone.Data.lidar.ranges[0], drone.Data.lidar.ranges[1], drone.Data.lidar.ranges[2], drone.Data.lidar.range_max);
            avoidance.orientation = orientationToObstacle(drone.Data.lidar.ranges[0], drone.Data.lidar.ranges[1], drone.Data.lidar.ranges[2], drone.Data.lidar.range_max, drone.Data.lidar.angle_max, drone.Data.lidar.angle_min);
            avoidance.orientation = degtorad(avoidance.orientation);

            // Output the distance to the obstacle and the orientation of the obstacle
            ROS_INFO("The distance to the obstacle is: [%f]; the inclination of the obstacle is: [%f]", avoidance.horizontal, avoidance.orientation);

            // Record that an obstacle has been detected
            FlSalam = 1;

            // Store the distances to the obstacle; needed afterwards to check if behind the obstacle there is another obstacle;
            // this vector retains the distances to the obstacles, thus creating a map; OVERSIMPLIFIED VERSION OF SLAM
            ObstacleDistance.left.push_back(drone.Data.lidar.ranges[0]);
            ObstacleDistance.centre.push_back(drone.Data.lidar.ranges[1]);
            ObstacleDistance.right.push_back(drone.Data.lidar.ranges[2]);
            ObstacleDistance.xPosition.push_back(avoidance.horizontal + drone.Data.local_pose.pose.position.x);

            // While the drone is between 2 obstacles perform an oblique climb
            while (drone.Data.local_pose.pose.position.x < (ObstacleDistance.xPosition[ObstacleDistance.xPosition.size() - 1] + ObstacleDistance.xPosition[ObstacleDistance.xPosition.size() - 2]) * 0.5)
            {
                if (detectIfOverObstacle(drone.Data.altitude.bottom_clearance, drone.Data.local_pose.pose.position.z) == 0 && drone.Data.lidar.ranges[0] < 10)
                {
                    // Not over obstacle, can go up in height
                    drone.Commands.move_Velocity_Local(0, 0.5, 0.25, 0, "BODY_OFFSET");
                    ROS_INFO("Oblique climb");
                }
                else
                {
                    // Over obstacle, do not go up in height
                    drone.Commands.move_Velocity_Local(0, 0.5, 0, 0, "BODY_OFFSET");
                    ROS_INFO("Move Forward");
                }
                ros::spinOnce();
                rate.sleep();
            }

        }
        // Else if there is no visible obstacle just go forwards at current height
        else
        {
            ROS_INFO("THERE IS NO VISIBLE OBSTACLE!");

            // Move forwards Command
            drone.Commands.move_Velocity_Local(0, 0.5, 0, 0, "BODY_OFFSET");
			ROS_INFO("Move Forward 2");
            //Store the distances to the obstacle as distances
            ObstacleDistance.left.push_back(drone.Data.lidar.range_max);
            ObstacleDistance.centre.push_back(drone.Data.lidar.range_max);
            ObstacleDistance.right.push_back(drone.Data.lidar.range_max);
        }

        // Reach a distances where the sensors do not detect anything using while conditions;
        while (drone.Data.lidar.ranges[0] != INFINITY && drone.Data.lidar.ranges[1] != INFINITY && drone.Data.lidar.ranges[2] != INFINITY)
        {
            // Move up until sensors do not detect anything
            drone.Commands.move_Velocity_Local(0, 0, 0.5, 0, "BODY_OFFSET");

            // Check if there is a new obstacle behind the current one; if so stop this while loop --> stop going up
            if (ObstacleDistance.left[ObstacleDistance.left.size() - 1] < 0.8 * drone.Data.lidar.ranges[0] &&
                    ObstacleDistance.centre[ObstacleDistance.centre.size() - 1] < 0.8 * drone.Data.lidar.ranges[1] &&
                    ObstacleDistance.right[ObstacleDistance.right.size() - 1] < 0.8 * drone.Data.lidar.ranges[2])
                break;

            // Store the height of the obstacle
            ObstacleDistance.yPosition.push_back(drone.Data.local_pose.pose.position.z);
            ros::spinOnce();
            rate.sleep();
        }

        // After finding out the height of the obstacle we have to account for bad sensor readings; go up another 25 cm
        while (drone.Data.local_pose.pose.position.z < ObstacleDistance.yPosition[ObstacleDistance.yPosition.size() - 1] + 0.35)
        {
            // Move up.
            drone.Commands.move_Velocity_Local(0, 0, 0.5, 0, "BODY_OFFSET");
            ROS_INFO("Climb");
            ros::spinOnce();
            rate.sleep();
        }

        while (drone.Data.local_pose.pose.position.x < 0.25)
        {
            // Go forward command
            drone.Commands.move_Velocity_Local(0, 1.0, 0.1, 0, "BODY_OFFSET");
            ROS_INFO("Slightly Oblique");
            ros::spinOnce();
            rate.sleep();
        }

        while (0.8 * drone.Data.local_pose.pose.position.z > drone.Data.altitude.bottom_clearance && FlSalam == 1)
        {
            ROS_INFO("Forward quickly.");
            drone.Commands.move_Velocity_Local(0, 1.0, 0, 0, "BODY_OFFSET");
            ros::spinOnce();
            rate.sleep();
        }
        ros::spinOnce();
        rate.sleep();
    }

    // Record positions at end of mission
    storePosition(POSITIONS.FinalObstacleAvoidance, drone.Data.local_pose.pose.position.x,
                  drone.Data.local_pose.pose.position.y, drone.Data.local_pose.pose.position.z,
                  drone.Data.compass_heading.data);




    ///<-------------------------------------------- STEP 3 ---- AMBULANCE PN TRACKING ---------------------------------------_>




    float setAltitude = 5.77f;

    // Start Mission; Initialize the relative velocities
    InitialiseJakeCode(drone.Data.target_position_relative.point.x, drone.Data.target_position_relative.point.y, drone.Data.target_position_relative.point.z);

    // Command 1, set drone velocity to the calculated initial velocity in 1 second.
    ROS_INFO("Initialising drone velocity");

    // Change this to a while loop comparing measured drone velocity and commanded drone velocity
    drone.Commands.Initialise_Velocity_for_AccelCommands(droneVel[1], droneVel[0], -droneVel[2]);

    float initial_yaw = atan2(droneVel[0], droneVel[1]) * 180.0 / M_PI;
    // turn drone to point in that direction
    ROS_INFO("Turning to direction of inital velocity: %f degrees", initial_yaw); // assume 10 degrees / second?
    for (int count = 1; count < 60; count++)
    {
        drone.Commands.move_Position_Local(0.0f, 0.0f, 0.0f, initial_yaw, "LOCAL_OFFSET", count);
        ros::spinOnce();
        rate.sleep();
    }

    // current yaw angle given by the accelerations
    float yawAcceleration;

    // buffer which stores actual and previous values of actual yaw
    boost::circular_buffer<float> yawError(2);
    yawError[0] = 0.0;
    yawError[1] = 0.0;

    // Kp declaration
    std::vector<float> kp;
    float kpCurrent;

    do
    {
        ROS_INFO("%f");
        droneAccComp(relPos, relVel, droneAcc);
        accFix = altitudeFix(drone.Data.target_position_relative.point.z, setAltitude);
        ROS_INFO("Accelerations needed: x: %f, y: %f, z: %f", droneAcc[1], droneAcc[0], droneAcc[2]);

        yawAcceleration = atan2(droneAcc[1], droneAcc[0]) * 180 / M_PI;
        yawError.push_back(yawAcceleration);
        kp.push_back(propControl(yawError[0], yawError[1], drone.Data.compass_heading.data));
        drone.Commands.move_Acceleration_Local_Trick(droneAcc[1], droneAcc[0], accFix, "LOCAL_OFFSET", loop_rate);



        for (int i = 0; i < 3; ++i)
        {
            relPosOld[i] = relPos[i];
        }

        relPos[0] = drone.Data.target_position_relative.point.y;
        relPos[1] = drone.Data.target_position_relative.point.x;
        relPos[2] = 0;

        gpsdistance = norm(relPos);

        ROS_INFO("Distance to target: %f", gpsdistance);

        velFromGPS(relPos, relPosOld, loop_rate, relVel);


        ros::spinOnce();
        rate.sleep();



    }
    while (gpsdistance > switchDist);







    ////<----------------------------------------------------GPS Landing------------------------------------------------------>
    ROS_INFO("Track Ambulance");
    for (int count = 1; count < 200; count++)
    {
        float yaw_angle = drone.Data.CalculateYawAngleToTarget();
        std::cout << "Yaw angle is: " << yaw_angle << " deg" << std::endl;
        // drone.Commands.move_Position_Global(drone.Data.gps_raw.latitude, drone.Data.gps_raw.longitude, drone.Data.gps_raw.altitude + 5.0f, yaw_angle, "BODY");
        drone.Commands.move_Velocity_Local_Gerald(1.50, yaw_angle, "BODY");
        ros::spinOnce();
        rate.sleep();
    }

    // Land and disarm
    ROS_INFO("Landing Now");
    drone.Commands.request_LandingAuto();


    return 0;
}














////<-------------------------------Function definitions----------------------->
void storePosition(struct MissionPositions STEP, float x, float y, float z, float yaw)
{
    STEP.x = x;
    STEP.y = y;
    STEP.z = z;
    STEP.yaw = yaw;
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

float degtorad(float a)
{
    return a * M_PI / 180;
}

float radtodeg(float a)
{
    return a * 180 / M_PI;
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

bool detectIfOverObstacle(float a, float b)
{
    if (a < 0.5 * b)
        return 1;
    else
        return 0;
}

float propControl(float aNow, float aOld, float b)
{
    float error = aNow;
    float diff = aNow - b;

    return error / diff;
}