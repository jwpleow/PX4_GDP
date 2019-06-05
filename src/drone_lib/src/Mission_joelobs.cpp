#include "headers/gdpdrone.h"
#include <cmath>

// bool detectObstacle(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int datawidth);


// struct Avoidance
// {
//     float horizontal, vertical;
//     float orientation;
// };

// struct gazeboposition
// {
//     float x, y, z; 
// };

// Avoidance avoidance;
// gazeboposition gazeboPosition;

int main(int argc, char **argv)
{
    // // Initialise node
    // ros::init(argc, argv, "joelobs_node");

    // // Create drone object, this sets everything up
    // GDPdrone drone;

    // // Set the rate. Default working frequency is 25 Hz
    // float loop_rate = 10.0;
    // ros::Rate rate = ros::Rate(loop_rate);

    // // Initialise and Arm
    // drone.Commands.await_Connection();
    // drone.Commands.set_Offboard();
    // drone.Commands.set_Armed();

    // // MISSION STARTS HERE:
    // // Request takeoff at 1m altitude. At 25Hz = 10 seconds
    // float altitude = 1.0;
    // int time_takeoff = 100;

    // // Initialize position 
    // gazeboPosition.x = 0; 
    // gazeboPosition.z = altitude; 

    // drone.Commands.request_Takeoff(altitude, time_takeoff);


    // int pointcloudwidth = drone.Data.depth_cam_pc2.width;
   



    // while (drone.Data.local_pose.pose.position.x < 10){

    //     pointcloudwidth = drone.Data.depth_cam_pc2.width;


    // if (detectObstacle(drone.Data.depth_cam_cloud, pointcloudwidth))
    // {
     
    //         // fly up till obstacle not detected
    //         while(detectObstacle(drone.Data.depth_cam_cloud, pointcloudwidth))
    //         {
    //             drone.Commands.move_Velocity_Local(0.0, 0.0, 0.5, 0, "BODY_OFFSET");
    //             ROS_INFO("Moving up.");
    //             ros::spinOnce();
    //             rate.sleep();
    //         }
            
    //         // move an extra bit up afterwards
    //         ROS_INFO("Moving an extra bit upwards");
    //         for (int i = 0; i < 50; i++){
    //             drone.Commands.move_Velocity_Local(0.0, 0.0, 0.5, 0, "BODY_OFFSET");
    //             ros::spinOnce();
    //             rate.sleep();
    //         }
    // }
    // else
    // {
    //     ROS_INFO("Moving Forward");
    //     drone.Commands.move_Velocity_Local(0.0, 1.5, 0, 0, "BODY_OFFSET");
    //     ros::spinOnce();
    //     rate.sleep();
    // }


    // }

    // // Land and disarm
    // drone.Commands.request_LandingAuto();

    return 0;
}

// bool detectObstacle(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int datawidth)
// {
//     bool isObstacle = 0;
//     int numObstaclePoints = 0;
//     ROS_INFO("width : %i", datawidth);

//     for(int i = 1; i < datawidth; i++) 
//     {
//         if(cloud->points[i].x < 3 && cloud->points[i].z> -0.3 && abs(cloud->points[i].y)< 0.3) 
//         {
            
//             numObstaclePoints += 1;
//         }
        
//         if (numObstaclePoints > 30){
//             ROS_INFO("obstacle distance x= %f, z= %f, index = %i, numobstacle = %i", cloud->points[i].x, cloud->points[i].z, i, numObstaclePoints);
//             isObstacle = 1; ///< set isObstacle if anything detected 3.5 away
//             break;
//         }
//     }
//     // && cloud->points[i].x!=NAN

//     return isObstacle;
// }
