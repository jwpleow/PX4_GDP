#ifndef COMMANDS_H
#define COMMANDS_H

#include <ros/ros.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/ExtendedState.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/GlobalPositionTarget.h>
#include <std_msgs/Float64.h>

class commands
{
public:
    //-----   PUBLIC METHODS -----//
    commands();
    commands(float _rate);

    void await_Connection();
    void set_Offboard();
    void set_Armed();
    void set_Disarmed();
    void request_Landing();
    void request_LandingAuto();
    void reset_Velocities();
    void request_Takeoff(float _altitude, float _counter);
    void request_Hover(float _time);

    void move_Position_Local(float _x, float _y, float _z, float _yaw_angle_deg, std::string _frame);
    void move_Velocity_Local(float _x, float _y, float _z, float _yaw_rate_deg_s, std::string _frame);
    void move_Acceleration_Local(float _x, float _y, float _z, std::string _frame);
    void move_Acceleration_Local_Trick(float _x, float _y, float _z, std::string _frame, int rate);
    void move_Position_Global(float _latitude, float _longitude, float _altitude, float _yaw_angle_deg, std::string _frame);


    ///< Overloaded for Silwood test 1 mission
    void move_Velocity_Local_geraldtest(float _fixed_speed, float _yaw_angle_deg, std::string _frame);

private:
    //-----   PRIVATE PROPERTIES -----//
    ros::NodeHandle nh;
    ros::Rate rate = ros::Rate(25.0);
    float velocity_x;
    float velocity_y;
    float velocity_z;
    float com_x;
    float com_y;
    float com_z;
    float com_yaw;
    std::vector<float> corrected_vector;

    //-----   PRIVATE DATA STORES -----//
    mavros_msgs::ExtendedState extended_state;
    mavros_msgs::State current_state;
    geometry_msgs::PoseStamped local_pose;
    std_msgs::Float64 compass_heading;

    //-----   PRIVATE CLIENTS -----//
    ros::ServiceClient set_mode_client;
    ros::ServiceClient arming_client;

    //-----   PRIVATE PUBLISHERS -----//
    ros::Publisher position_pub;
    ros::Publisher velocity_pub;
    ros::Publisher acceleration_pub;
    ros::Publisher target_pub_local;
    ros::Publisher target_pub_global;
    ros::Publisher target_pub_global_raw;

    //-----   PRIVATE SUBSCRIBERS -----//
    ros::Subscriber state_sub;
    ros::Subscriber state_sub_ext;
    ros::Subscriber pose_sub;
    ros::Subscriber compass_sub;

    //-----   PRIVATE METHODS -----//
    void set_Mode(std::string _mode);
    void set_Arm_Disarm(bool _arm);
    void ext_state_cb(const mavros_msgs::ExtendedState::ConstPtr &msg);
    void state_cb(const mavros_msgs::State::ConstPtr &msg);
    void pose_cb(const geometry_msgs::PoseStamped::ConstPtr &msg);
    void heading_cb(const std_msgs::Float64::ConstPtr& msg); 
    void set_frame(mavros_msgs::PositionTarget *_pos, std::string _frame, bool velocity_acc);
    std::vector<float> transform_frame(std::vector<float> _vector, std::string _frame);
    bool check_Inputs(float _x, float _y, float _z, float yaw);
};

#endif