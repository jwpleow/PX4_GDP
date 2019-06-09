#include <opencv2/core.hpp>

#include "tracker-arb/TrackerARB.h"

///< ROS headers
#include "ros/ros.h"
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Bool.h>

#define DEFAULT_PORT 0
#define VID_CAPTURE_WIDTH 640

std_msgs::Bool bool_msg;
geometry_msgs::Twist data_msg;

using namespace cv;
using namespace std;

int main(int argc, char **argv) {
  // Initialise ROS with node name
  ros::init(argc, argv, "vishnu_cam_node");
  ros::NodeHandle n;
  ros::Rate rate(10);
  
  // Publish to topic /vishnu_cam_data
  ros::Publisher vishnu_cam_data_pub = n.advertise<geometry_msgs::Twist>("vishnu_cam_data", 10);
  // Publish whether cam is detecting the ARtag
  ros::Publisher vishnu_cam_detection_pub = n.advertise<std_msgs::Bool>("vishnu_cam_detection", 10);
  
  const float markerLength = 3.70;
  const float markerSeparation = 8.70;
  const int markersXY = 2;
  CVCalibration cvl("CalibParams.txt");
  TrackerARB tracker(cvl, markerLength, markerSeparation, markersXY, true);
  
  int port = argc > 1 ? stoi(argv[1]) : DEFAULT_PORT;
  
  Mat frame;
  Vec3d tVec, rVec, ctVec;
  VideoCapture vid(port);
  vid.set(CAP_PROP_FRAME_WIDTH, VID_CAPTURE_WIDTH);
  ROS_INFO("Width: %f, Height: %f", vid.get(CAP_PROP_FRAME_WIDTH), vid.get(CAP_PROP_FRAME_HEIGHT));
  while(true) {
    if(!vid.read(frame)) {
      break;
      ROS_INFO("Unable to read video frame");
    }
    if (tracker.getPose(frame, tVec, rVec)) {
      tracker.correctedPose(rVec, tVec, ctVec);
      ROS_INFO("tVec_x: %f, tVec_y: %f, tVec_z: %f", ctVec[0], ctVec[1], ctVec[2]);
      data_msg.linear.x   = (float) (ctVec[0] /  100);
      data_msg.linear.y   = (float) (ctVec[1] / 100);
      data_msg.linear.z   = (float) (ctVec[2] / 100);
      data_msg.angular.x  = (float) rVec[0];
      data_msg.angular.y  = (float) rVec[1];
      data_msg.angular.z  = (float) rVec[2];
      vishnu_cam_data_pub.publish(data_msg);

      bool_msg.data = 1;
    }
    else{
      bool_msg.data = 0;
    }
    vishnu_cam_detection_pub.publish(bool_msg);
    ros::spinOnce();
  }
  return 0;
}