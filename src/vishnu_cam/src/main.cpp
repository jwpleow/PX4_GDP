#include <opencv2/core.hpp>

#include "tracker-ar/TrackerAR.h"

///< ROS headers
#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>

ros::Publisher vishnu_cam_data_pub;
geometry_msgs::Twist data_msg;

using namespace cv;
using namespace std;

int main(int argc, char **argv) {

    // Initialise ROS with node name
    ros::init(argc, argv, "vishnu_cam_node");
    ros::NodeHandle n;
    ros::Rate rate(10);

    // Publish to topic /vishnu_cam_data
    vishnu_cam_data_pub = n.advertise<geometry_msgs::Twist>("vishnu_cam_data", 10);



    const auto arucoSquareDimension = 0.0370f;
    CVCalibration cvl("CalibParams.txt");
    TrackerAR tracker(cvl, arucoSquareDimension);
    
    int port = 0;
    if (argc>1) port = stoi(argv[1]);

    Mat frame;
    Vec3d tVec, rVec;
    VideoCapture vid(port);

    while(true) {
        if(!vid.read(frame)) {
            break;
            cerr << "Unable to read video frame\n";
        }
        if (tracker.getPose(frame, tVec, rVec)) {
            data_msg.linear.x = tVec[2];
            data_msg.linear.y = tVec[1];
            data_msg.linear.z = tVec[0];
            data_msg.angular.x = tVec[2];
            data_msg.angular.y = tVec[1];
            data_msg.angular.z = tVec[0];

            vishnu_cam_data_pub.publish(data_msg);
            ros::spinOnce();
        }
    }
    
    tracker.startStreamingTrack(port);








    return 0;
}
