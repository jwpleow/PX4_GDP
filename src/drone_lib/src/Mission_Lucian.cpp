#include "lucian/Dstar.h"
#include "headers/gdpdrone.h"

int xobst = 0, yobst = 0; // global xobst/yobst value
const float sensor_angle = 49.35;

int approx(float a)
{
    int A = (int)floor(a * 10);
    if (A % 2 == 1)
        return --A;
    else
        return A;
}

double CalculatePitch(float x, float y, float z, float w);

int ObstacleCheck(float pitch, std::vector<float> ranges);

int Calculate_Y_Obst();

int main(int argc, char **argv)
{
    // Initialise node
    ros::init(argc, argv, "Bukake_node");

    GDPdrone drone;

    // Set the rate. Default working frequency is 25 Hz
    float loop_rate = 25.0;
    ros::Rate rate = ros::Rate(loop_rate);

    // Initialise and Arm
    drone.Commands.await_Connection();
    drone.Commands.set_Offboard();
    drone.Commands.set_Armed();

    Dstar *dstar = new Dstar();
    list<state> mypath;

    // MISSION STARTS HERE:
    // Request takeoff at 1m altitude. At 25Hz = 10 seconds
    float altitude = 1.0;
    int time_takeoff = 100;
    drone.Commands.request_Takeoff(altitude, time_takeoff);

    float pitch = 0;


    int Count;

    ///< if nothing detected for top sensors, and OK for middle bottom sensor, move forward
    while (ObstacleCheck(pitch, drone.Data.lidar.ranges) == 0)
    {
        drone.Commands.move_Velocity_Local(0, 0.5, 0, 0, "BODY_OFFSET"); ///< replace this with jake PN bit in final
        pitch = CalculatePitch(drone.Data.imu.orientation.x, drone.Data.imu.orientation.y, drone.Data.imu.orientation.z, drone.Data.imu.orientation.w);
        ROS_INFO("Pitch: %f", pitch);
        ros::spinOnce();
        rate.sleep();
    }
    pitch = CalculatePitch(drone.Data.imu.orientation.x, drone.Data.imu.orientation.y, drone.Data.imu.orientation.z, drone.Data.imu.orientation.w);
    ///< if something detected only by bottom middle laser, go up slightly
    if (ObstacleCheck(pitch, drone.Data.lidar.ranges) == 1)
    {
        for (int count = 1; count < 21; count++)
        {
            drone.Commands.move_Position_Local(0, 0, 0.2, 0, "BODY_OFFSET", Count);
            ros::spinOnce();
            rate.sleep();
        }
    }

    ROS_INFO("Moved to main loop");
    ROS_INFO("xobst [%i]", xobst);
    ROS_INFO("yobst [%i]", yobst);

    int distance = 400; //gps distance in m between goal and where I am
    int resolution = 2; //Resolution is 2/10 m;
    int Altitude = (int)altitude;

    dstar->init(0, Altitude, distance / resolution, 0);
    for (int i = -distance / resolution; i <= distance / resolution; i++)
        dstar->updateCell(i, -1, -1); //set the ground to be non-traversable

    for (int i = 0; i <= yobst / resolution + 1; i++)
        dstar->updateCell(xobst / resolution, i, -1);

    int xst = 0, yst = 0;

    while (!(xst == distance / resolution && yst == 0))
    {
        dstar->replan();           // plan a path
        mypath = dstar->getPath(); // retrieve path

        for (auto &v : mypath)
        {
            dstar->updateStart(v.x, v.y); // move start to new path point
            ROS_INFO("v.x [%i]", v.x);
            ROS_INFO("v.y [%i]", v.y);

            for (int count = 1; count < 21; count++)
            {
                drone.Commands.move_Position_Local(0, (v.x - xst) * 0.2, (v.y - yst) * 0.2, 0, "BODY_OFFSET", Count); ///< move to path next path step
                Count++;
                ros::spinOnce();
                rate.sleep();
            }
            xst = v.x;
            yst = v.y;
            if (v.x == xobst / resolution && v.y == (yobst / resolution + 2))
            {
                ROS_INFO("COITE");
                ///< if nothing detected for top sensors, and OK for middle bottom sensor, move forward
                while (ObstacleCheck(pitch, drone.Data.lidar.ranges) == 0)
                {
                    drone.Commands.move_Velocity_Local(0, 0.5, 0, 0, "BODY_OFFSET"); ///< replace this with jake PN bit in final
                    pitch = CalculatePitch(drone.Data.imu.orientation.x, drone.Data.imu.orientation.y, drone.Data.imu.orientation.z, drone.Data.imu.orientation.w);
                    ROS_INFO("Pitch: %f", pitch);
                    ros::spinOnce();
                    rate.sleep();
                }
                pitch = CalculatePitch(drone.Data.imu.orientation.x, drone.Data.imu.orientation.y, drone.Data.imu.orientation.z, drone.Data.imu.orientation.w);
                ///< if something detected only by bottom middle laser, go up slightly
                if (ObstacleCheck(pitch, drone.Data.lidar.ranges) == 1)
                {
                    for (int count = 1; count < 21; count++)
                    {
                        drone.Commands.move_Position_Local(0, 0, 0.2, 0, "BODY_OFFSET", Count);
                        ros::spinOnce();
                        rate.sleep();
                    }
                }

                ROS_INFO("xobst [%i]", xobst);
                ROS_INFO("yobst [%i]", yobst);

                int Count;
                int distance = 100; //gps distance in m between goal and where I am
                int resolution = 2; //Resolution is 2/10 m;
                int Altitude = (int)altitude;

                dstar->init(0, Altitude, distance / resolution, 0);
                for (int i = -distance / resolution; i <= distance / resolution; i++)
                    dstar->updateCell(i, -1, -1); //set the ground to be non-traversable

                for (int i = 0; i <= yobst / resolution + 1; i++)
                    dstar->updateCell(xobst / resolution, i, -1);

                int xst = 0, yst = 0;
                ROS_INFO("AM COAIELE MARI");
                break;
            }
            ros::spinOnce();
            rate.sleep();
        }
    }
    // Land and disarm
    drone.Commands.request_LandingAuto();
    return 0;
}


double CalculatePitch(float x, float y, float z, float w) ///< takes quaternions and returns pitch in degrees
{
    // pitch (y-axis rotation)
    float pitch;
    float sinp = +2.0 * (w * y - z * x);
    if (fabs(sinp) >= 1)
        pitch = copysign(M_PI / 2, sinp);
    else
        pitch = asin(sinp);


    return pitch * 180 / M_PI;

}

int ObstacleCheck(float pitch, std::vector<float> ranges) ///< functions returns 0 if no obstacle, 1 if too close, 2 if obstacle detected with top sensors, and, for (2), calculates their x distance
{

    ///< Check for obstacle
    if (!(ranges[3] == INFINITY && ranges[4] == INFINITY && ranges[5] == INFINITY && ranges[1] > 0.2))
    {

        if(ranges[1] < 0.2) ///< this is if something is far too close
        {
            return 1;
        }
        ///< If something detected by top middle lidar, use that for yobst, and return obstacle = 2
        else if (ranges[4] != INFINITY)
        {
            yobst = approx(ranges[4] * sin((sensor_angle - pitch) * M_PI / 180));
            xobst = approx(ranges[1] * cos(pitch * M_PI / 180));
            return 2;
        }
        else if (ranges[5] != INFINITY)
        {
            yobst = approx(ranges[5] * sin((sensor_angle - pitch) * M_PI / 180)); ///< fudge as laser is not straight
            xobst = approx(ranges[1] * cos(pitch * M_PI / 180));
            return 2;
        }
        else
        {
            yobst = approx(ranges[3] * sin((sensor_angle - pitch) * M_PI / 180));
            xobst = approx(ranges[1] * cos(pitch * M_PI / 180));
            return 2;
        }
    }
    return 0; ///< 0 if no obstacle
}
