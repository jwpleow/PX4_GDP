#include "headers/commands.h"
#include "headers/functions.h"
#include "headers/data.h"

//-----   METHODS -----//
// Default constructor
commands::commands(float _rate)
{
    rate = ros::Rate(_rate);

    // Reset Accelerations
    velocity_x = 0;
    velocity_y = 0;
    velocity_z = 0;

    // Subscribers
    state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, boost::bind(&commands::state_cb, this, _1));
    state_sub_ext = nh.subscribe<mavros_msgs::ExtendedState>("mavros/extended_state", 10, &commands::ext_state_cb, this);

    // Publishers
    position_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
    velocity_pub = nh.advertise<geometry_msgs::Twist>("mavros/setpoint_velocity/cmd_vel_unstamped", 10);
    acceleration_pub = nh.advertise<geometry_msgs::Vector3Stamped>("mavros/setpoint_accel/accel", 10);
    target_pub_local = nh.advertise<mavros_msgs::PositionTarget>("mavros/setpoint_raw/local", 10);
    target_pub_global = nh.advertise<mavros_msgs::GlobalPositionTarget>("mavros/setpoint_position/global", 10);
    target_pub_global_raw = nh.advertise<mavros_msgs::GlobalPositionTarget>("mavros/setpoint_raw/global", 10);

    // Service clients
    arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
}

// Wait for connection from the drone
void commands::await_Connection()
{
    while (ros::ok() && !current_state.connected)
    {
        ros::spinOnce();
        rate.sleep();
    }

    ROS_INFO("GDPdrone: Connected");
}

void commands::set_Offboard()
{
    set_Mode("OFFBOARD");
}

void commands::set_Armed()
{
    set_Arm_Disarm(true);
}

void commands::set_Disarmed()
{
    set_Arm_Disarm(false);
}

void commands::request_LandingAuto()
{
    ROS_INFO("GDPdrone: Autonomous Landing Requested");
    set_Mode("AUTO.LAND");
}

void commands::request_Landing()
{
    // This will trigger the landed failsafe and switch off the motors
    ROS_INFO("GDPdrone: Landing Requested");
    geometry_msgs::Twist velocity;
    velocity = functions::make_twist(0, 0, -0.3, 0, 0, 0);

    // TODO: extended state does not work
    while ((ros::ok() && extended_state.landed_state != 1))
    {
        velocity_pub.publish(velocity);
    }
}

void commands::request_Takeoff(float _altitude, float _counter)
{
    ROS_INFO("GDPdrone: Takeoff Requested");
    for (int i = _counter; i > 0; i--)
    {
        move_Position_Local(0, 0, _altitude, 0, "LOCAL");
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("GDPdrone: Takeoff Completed");
}

//-----   MOVEMENT COMMANDS -----//
void commands::move_Position_Local(float _x, float _y, float _z, float _yaw_angle_deg, std::string _frame)
{
    mavros_msgs::PositionTarget pos;
    commands::make_frame_local(&pos, _frame);

    pos.type_mask = mavros_msgs::PositionTarget::IGNORE_VX | mavros_msgs::PositionTarget::IGNORE_VY |
                    mavros_msgs::PositionTarget::IGNORE_VZ | mavros_msgs::PositionTarget::IGNORE_AFX |
                    mavros_msgs::PositionTarget::IGNORE_AFY | mavros_msgs::PositionTarget::IGNORE_AFZ |
                    mavros_msgs::PositionTarget::FORCE | mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
    pos.position.x = _x;
    pos.position.y = _y;
    pos.position.z = _z;
    pos.yaw = functions::DegToRad(_yaw_angle_deg);
    target_pub_local.publish(pos);
}

void commands::move_Velocity_Local(float _x, float _y, float _z, float _yaw_rate_deg_s, std::string _frame)
{
    mavros_msgs::PositionTarget pos;
    commands::make_frame_local(&pos, _frame);

    pos.type_mask = mavros_msgs::PositionTarget::IGNORE_PX | mavros_msgs::PositionTarget::IGNORE_PY |
                    mavros_msgs::PositionTarget::IGNORE_PZ | mavros_msgs::PositionTarget::IGNORE_AFX |
                    mavros_msgs::PositionTarget::IGNORE_AFY | mavros_msgs::PositionTarget::IGNORE_AFZ |
                    mavros_msgs::PositionTarget::FORCE | mavros_msgs::PositionTarget::IGNORE_YAW;
    pos.velocity.x = _x;
    pos.velocity.y = _y;
    pos.velocity.z = _z;
    pos.yaw_rate = functions::DegToRad(_yaw_rate_deg_s);
    target_pub_local.publish(pos);
}

void commands::move_Acceleration_Local(float _x, float _y, float _z, std::string _frame)
{
    if (_x > 1.0 || _y > 1.0 || _z > 1.0)
    {
        ROS_INFO("Acceleration commands should be given as a percentage of thrust, from 0 to 1");
        return;
    }

    mavros_msgs::PositionTarget pos;

    pos.coordinate_frame = mavros_msgs::PositionTarget::FRAME_BODY_NED;
    pos.type_mask = mavros_msgs::PositionTarget::IGNORE_PX | mavros_msgs::PositionTarget::IGNORE_PY |
                    mavros_msgs::PositionTarget::IGNORE_PZ | mavros_msgs::PositionTarget::IGNORE_VX |
                    mavros_msgs::PositionTarget::IGNORE_VY | mavros_msgs::PositionTarget::IGNORE_VZ |
                    mavros_msgs::PositionTarget::IGNORE_YAW | mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
    pos.acceleration_or_force.x = _x;
    pos.acceleration_or_force.y = _y;
    pos.acceleration_or_force.z = _z;
    target_pub_local.publish(pos);
}

void commands::move_Acceleration_Local_Trick(float _x, float _y, float _z, std::string _frame, int rate)
{
    mavros_msgs::PositionTarget pos;
    commands::make_frame_local(&pos, _frame);

    pos.type_mask = mavros_msgs::PositionTarget::IGNORE_PX | mavros_msgs::PositionTarget::IGNORE_PY |
                    mavros_msgs::PositionTarget::IGNORE_PZ | mavros_msgs::PositionTarget::IGNORE_AFX |
                    mavros_msgs::PositionTarget::IGNORE_AFY | mavros_msgs::PositionTarget::IGNORE_AFZ |
                    mavros_msgs::PositionTarget::FORCE | mavros_msgs::PositionTarget::IGNORE_YAW |
                    mavros_msgs::PositionTarget::IGNORE_YAW_RATE;

    // Update accumulated velocites. Acc = DV/Dt -> DV = Acc * Dt -> DV = Acc / Freq
    velocity_x += _x / rate;
    velocity_y += _y / rate;
    velocity_z += _z / rate;

    // Send accumulated velocities
    pos.velocity.x = velocity_x;
    pos.velocity.y = velocity_y;
    pos.velocity.z = velocity_z;

    // Publish command
    target_pub_local.publish(pos);
}

void commands::move_Acceleration_Local(float _x, float _y, float _z)
{
    geometry_msgs::Vector3Stamped acceleration = functions::make_acceleration(_x, _y, _z);
    acceleration_pub.publish(acceleration);
}

void commands::move_Position_Global(float _latitude, float _longitude, float _altitude, float _yaw_angle_deg, std::string _frame)
{
    // This function uses GPS to reach a target GPS location

    mavros_msgs::GlobalPositionTarget pos;
    pos.header.stamp = ros::Time::now();

    pos.coordinate_frame = mavros_msgs::GlobalPositionTarget::FRAME_GLOBAL_INT;
    pos.type_mask = mavros_msgs::GlobalPositionTarget::IGNORE_VX | mavros_msgs::GlobalPositionTarget::IGNORE_VY |
                    mavros_msgs::GlobalPositionTarget::IGNORE_VZ | mavros_msgs::GlobalPositionTarget::IGNORE_AFX |
                    mavros_msgs::GlobalPositionTarget::IGNORE_AFY | mavros_msgs::GlobalPositionTarget::IGNORE_AFZ |
                    mavros_msgs::GlobalPositionTarget::FORCE | mavros_msgs::GlobalPositionTarget::IGNORE_YAW;

    pos.latitude = _latitude;
    pos.longitude = _longitude;
    pos.altitude = _altitude;
    pos.yaw = functions::DegToRad(_yaw_angle_deg);
    target_pub_global.publish(pos);
}

void commands::reset_Velocities()
{
    velocity_x = 0;
    velocity_y = 0;
    velocity_z = 0;
}

//-----   PRIVATE -----//
void commands::set_Mode(std::string _mode)
{
    mavros_msgs::SetMode mode;
    mode.request.custom_mode = _mode;

    ros::Time last_request = ros::Time::now();
    while (ros::ok() && current_state.mode != _mode)
    {
        if (ros::Time::now() - last_request > ros::Duration(0.25))
        {
            acceleration_pub.publish(functions::make_acceleration(0, 0, 0));
            set_mode_client.call(mode);
            last_request = ros::Time::now();
            ros::spinOnce();
            rate.sleep();
        }
    }
    if (current_state.mode == _mode)
    {
        ROS_INFO("GDPdrone: %s mode enabled", _mode.c_str());
    }
}

void commands::set_Arm_Disarm(bool _arm)
{
    mavros_msgs::CommandBool arm_command;
    arm_command.request.value = _arm;
    ros::Time last_request = ros::Time::now();
    while (ros::ok())
    {
        if (!current_state.armed && (ros::Time::now() - last_request > ros::Duration(0.25)))
        {
            if (arming_client.call(arm_command) && arm_command.response.success && _arm)
            {
                ROS_INFO("Vehicle Armed");
                break;
            }
            else if (arming_client.call(arm_command) && arm_command.response.success && !_arm)
            {
                ROS_INFO("Vehicle Disarmed");
                break;
            }
            last_request = ros::Time::now();
        }
    }
}

//-----   CALLBACKS -----//
// State subscriber callback function
void commands::ext_state_cb(const mavros_msgs::ExtendedState::ConstPtr &msg)
{
    extended_state = *msg;
}

// State subscriber callback function
void commands::state_cb(const mavros_msgs::State::ConstPtr &msg)
{
    current_state = *msg;
}

void commands::make_frame_local(mavros_msgs::PositionTarget *_pos, std::string _frame)
{
    // C++ does not support switching with strings, ugly "if" cascade intead
    if (_frame == "BODY")
    {
        _pos->coordinate_frame = mavros_msgs::PositionTarget::FRAME_BODY_NED;
    }
    else if (_frame == "LOCAL")
    {
        _pos->coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    }
    else if (_frame == "BODY_OFFSET")
    {
        _pos->coordinate_frame = mavros_msgs::PositionTarget::FRAME_BODY_OFFSET_NED;
    }
    else if (_frame == "LOCAL_OFFSET")
    {
        _pos->coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_OFFSET_NED;
    }
    else
    {
        ROS_INFO("Frame of reference should either be BODY or LOCAL");
        return;
    }
}