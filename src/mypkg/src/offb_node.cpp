#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <iostream>

#include "ros/ros.h"

#include "sensor_msgs/LaserScan.h"
#include "mavros_msgs/Altitude.h"
#include "mavros_msgs/State.h"
#include "mavros_msgs/SetMode.h"
#include "geometry_msgs/PoseStamped.h"
#include "mavros_msgs/CommandBool.h"

mavros_msgs::State current_state;


void CallBackCurrentState(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

class FrontLiDAR {
public:
     int length;
     double left={11.0};       // left sensor TeraRanger Tower
     double centre={11.0};     // centre sensor TeraRanger Tower
     double right={11.0};      // right sensor TeraRanger Tower
     float left_average = 0.0;
     float centre_average = 0.0;
     float right_average = 0.0;

     FrontLiDAR();                         // constructor
     ~FrontLiDAR();                        // destructor
};

FrontLiDAR::FrontLiDAR(){

}

FrontLiDAR::~FrontLiDAR(){

}

FrontLiDAR data;

void CallBackFrontLiDAR (const sensor_msgs::LaserScan::ConstPtr& scan)
{

    //ROS_INFO("Range of left: [%f]", scan->ranges[0]);
    //ROS_INFO("Range of centre: [%f]", scan->ranges[1]);
    //ROS_INFO("Range of right: [%f]", scan->ranges[2]);

    data.left = scan->ranges[0];
    data.centre = scan->ranges[1];
    data.right = scan->ranges[2];


}

class HeightLiDAR {
public:
    int length;
    double height = {0.0};
    float height_average = 0.0;

    HeightLiDAR();
    ~HeightLiDAR();
};

HeightLiDAR::HeightLiDAR(){
}
HeightLiDAR::~HeightLiDAR(){
}


HeightLiDAR data_h;
void CallBackHeightLiDAR(const mavros_msgs::Altitude::ConstPtr& scan)
{
    //ROS_INFO("Height of the drone is: [%f]", scan->bottom_clearance);
    data_h.height = scan->bottom_clearance;

}

double obstacleDistance_4m;

int main(int argc, char **argv)
{

    ros::init(argc, argv, "sensor_subscriber");

    ros::NodeHandle n;

    ros::Subscriber sub3 = n.subscribe("/laser/scan", 1000, CallBackFrontLiDAR);
    ros::Subscriber sub4 = n.subscribe("/mavros/altitude", 1000, CallBackHeightLiDAR);
    ros::Publisher local_pos_pub = n.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 100);
    ros::ServiceClient arming_client = n.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = n.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    geometry_msgs::PoseStamped pose1;
    pose1.pose.position.x = 0;
    pose1.pose.position.y = 0;
    pose1.pose.position.z = 0.5;

    geometry_msgs::PoseStamped pose2;
    pose2.pose.position.x = pose1.pose.position.x;
    pose2.pose.position.y = pose1.pose.position.y;
    pose2.pose.position.z = pose1.pose.position.z;

    //Send a few setpoints before starting Before entering Offboard mode, you must have already
    //started streaming setpoints. Otherwise the mode switch will be rejected.
    //Here, 100 was chosen as an arbitrary amount.

    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(pose1);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();
    float diff = 1;

    while(ros::ok() && diff>0.30){
        if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }

        local_pos_pub.publish(pose1);
        diff = pose1.pose.position.z - data_h.height_average;

        ros::spinOnce();
        rate.sleep();
    }

    // Get of the ground until sensors do not detect anything anymore
    int counter = 0;
    while(ros::ok()){
        while (data.left < 4 || data.right < 4 || data.centre < 4)
        {
            pose2.pose.position.z += 0.01;
            local_pos_pub.publish(pose2);
            ros::spinOnce();
            rate.sleep();
        }
    }

    return 0;
}


    /* Ocillate between poses
    int counter = 0;
    while(ros::ok()){
        while(ros::ok() && counter < 100){
            local_pos_pub.publish(pose2);
            ros::spinOnce();
            rate.sleep();
            counter++;
        }

        counter = counter * -1;

        while(ros::ok() && counter < 0){
            local_pos_pub.publish(pose3);
            ros::spinOnce();
            rate.sleep();
            counter++;
        }
    }

    return 0;
}*/
