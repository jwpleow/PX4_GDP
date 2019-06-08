#include "headers/data.h"

const double pi = 3.14159265358979;

data::data(float _rate)
{
    rate = ros::Rate(_rate);

    // Default, don't store data
    save_data = false; 

    // Subscribe to Altitude Data                                                       ///< e.g. drone.Data.altitude.bottom_clearance
    altitude_sub = nh.subscribe<mavros_msgs::Altitude>("/mavros/altitude", 10, &data::altitude_cb, this);

    // Subscribe to Compass Data
    compass_sub = nh.subscribe<std_msgs::Float64>("/mavros/global_position/compass_hdg", 10, &data::heading_cb, this);

    // Subscribe to GPS Data
    gps_sub = nh.subscribe<sensor_msgs::NavSatFix>("/mavros/global_position/global", 10, &data::gps_cb, this);

    // Subscribe to LiDar Data
    lidar_sub = nh.subscribe<sensor_msgs::LaserScan>("/laser/scan", 10, &data::lidar_cb, this);

    // Subscribe to IMU Data
    imu_sub = nh.subscribe<sensor_msgs::Imu>("/mavros/imu/data", 10, &data::imu_cb, this);

    // Subscribe to Position Data                                                       ///< in local coordinates ENU, example: drone.Data.local_pose.linear.x
    pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 10, &data::pose_cb, this);

    // Subscribe to Velocity Data                                                       ///< in local coordinates ENU, example: drone.Data.local_velocity.linear.x
    velocity_sub = nh.subscribe<geometry_msgs::TwistStamped>("/mavros/local_position/velocity", 10, &data::velocity_cb, this);

    ///< Subscribe to target xyz relative to drone                                      ///< ENU - e.g. drone.Data.target_position_relative.point.x
    target_position_relative_sub = nh.subscribe<geometry_msgs::PointStamped>("/filtered_target_wrtdrone_position", 1, &data::target_position_relative_cb, this);
 
    ///< Subscribe to target abs velocity                                                ///< ENU - e.g. drone.Data.target_abs_velocity.linear.x
    target_abs_velocity_sub = nh.subscribe<geometry_msgs::TwistStamped>("/filtered_target_abs_velocity", 1, &data::target_abs_velocity_cb, this);

    ///< Subscribe to target GPS data
    target_gps_sub = nh.subscribe<sensor_msgs::NavSatFix>("/android/fix", 1, &data::target_gps_cb, this);

    ///< Subscribe to vishnu cam data                                                   ///< Get body coordinates (RIGHT,DOWN,UP) of ARtag using e.g. drone.Data.vishnu_cam_data.linear.x
    vishnu_cam_data_sub = nh.subscribe<geometry_msgs::Twist>("/vishnu_cam_data", 1, &data::vishnu_cam_data_cb, this);

    ///< Subscribe to vishnu cam detection                                              ///< Check if vishnu cam detects ARtag using drone.Data.vishnu_cam_detection.data == 1
    vishnu_cam_detection_sub = nh.subscribe<std_msgs::Bool>("/vishnu_cam_detection", 1, &data::vishnu_cam_detection_cb, this);
  
}

///< Yaw angle calculator (in degrees) based off target position relative to drone
// - code causes a quarternion break when over the target sometimes
float data::CalculateYawAngleToTarget()
{
    return atan2(target_position_relative.point.y, target_position_relative.point.x) * 180.0 / pi;
}


///< Vishnu cam data callback function
void data::vishnu_cam_data_cb(const geometry_msgs::Twist::ConstPtr &msg)
{
    vishnu_cam_data = *msg;
}

///< Vishnu cam detection boolean callback function
void data::vishnu_cam_detection_cb(const std_msgs::Bool::ConstPtr &msg)
{
    vishnu_cam_detection = *msg;
}

///< Target absolute velocity callback function
void data::target_abs_velocity_cb(const geometry_msgs::TwistStamped::ConstPtr &msg)
{
    target_abs_velocity = *msg;
}

///< Target position relative callback function
void data::target_position_relative_cb(const geometry_msgs::PointStamped::ConstPtr &msg)
{
    target_position_relative = *msg;
}

///< Target GPS subscriber callback function
void data::target_gps_cb(const sensor_msgs::NavSatFix::ConstPtr &msg)
{
    target_gps = *msg;
}

// Altitude subscriber callback function
void data::altitude_cb(const mavros_msgs::Altitude::ConstPtr &msg)
{
    altitude = *msg;
    if (save_data){bag.write("/mavros/altitude", ros::Time::now(), *msg);}
}

// Heading subscriber callback function
void data::heading_cb(const std_msgs::Float64::ConstPtr &msg)
{
    compass_heading = *msg;
    if (save_data){bag.write("/mavros/global_position/compass_hdg", ros::Time::now(), *msg);}
}

// GPS subscriber callback function
void data::gps_cb(const sensor_msgs::NavSatFix::ConstPtr &msg)
{
    gps_raw = *msg;
    if (save_data){bag.write("/mavros/global_position/global", ros::Time::now(), *msg);}
}

// LiDar subscriber callback function
void data::lidar_cb(const sensor_msgs::LaserScan::ConstPtr &msg)
{
    lidar = *msg;
}

// IMU subscriber callback function
void data::imu_cb(const sensor_msgs::Imu::ConstPtr &msg)
{
    imu = *msg;
}

// Pose subscriber callback function
void data::pose_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    local_pose = *msg;
    if (save_data){bag.write("/mavros/local_position/pose", ros::Time::now(), *msg);}
}

// Velocity subscriber callback function
void data::velocity_cb(const geometry_msgs::TwistStamped::ConstPtr &msg)
{
    local_velocity = *msg;
}

std::string data::get_log_name()
{
    auto t = std::time(nullptr);
    auto tm = *std::localtime(&t);

    std::ostringstream oss;
    oss << std::put_time(&tm, "%d-%m-%Y %H-%M-%S");
    return "src/drone_lib/flight_data/" + oss.str() + "_data.bag";
}

void data::start_rosbag()
{
    // Initialise Ros Bag
    bag.open(get_log_name(), rosbag::bagmode::Write);
    save_data = true; 
}

