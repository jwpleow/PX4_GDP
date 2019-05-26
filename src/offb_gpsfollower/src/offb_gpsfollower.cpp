#include <cstdlib>
#include "drone_offboard.h"


int main(int argc, char** argv)
{
  ros::init(argc, argv, "offb_gpsfollower_node");
  ros::Time::init();
  Drone aero;
  int ret = EXIT_SUCCESS;
  bool is_success;

  // Arm
  if (!aero.arm())
  {
    ROS_ERROR("Fatal: Arming failed!");
    ret = EXIT_FAILURE;
    goto end;
  }

  // Takeoff
  if (!aero.takeoff())
  {
    ROS_ERROR("Fatal: Takeoff failed!");
    ret = EXIT_FAILURE;
    goto end;
  }
  else
  {
    ROS_INFO("Takeoff sent");
  }
  sleep(5);  // Let Aero reach takeoff altitude


if (aero.setOffboardMode())
    ROS_INFO("Offboard enabled");
  else
  {
    ROS_ERROR("Unable to switch to Offboard");
    return false;
  }

while (ros::ok()){


 aero.FlyToGlobalPosTarget();
 ros::spinOnce();
 aero.rate_.sleep();

}


return 0;


end:
  ros::shutdown();
  return ret;
}

