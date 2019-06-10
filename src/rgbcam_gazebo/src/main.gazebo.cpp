#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "tracker-arb/TrackerARB.h"
#include <opencv2/core.hpp>
///< ROS headers
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Bool.h>




cv::Vec3d tVec, rVec, ctVec;
const float markerLength = 3.70;
const float markerSeparation = 8.70;
const int markersXY = 2;
CVCalibration cvl("CalibParams.txt");
TrackerARB tracker(cvl, markerLength, markerSeparation, markersXY, markersXY, true);


ros::Publisher vishnu_cam_data_pub;
ros::Publisher vishnu_cam_detection_pub;

std_msgs::Bool bool_msg;
geometry_msgs::Twist data_msg;

class ImageConverter
{
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;


public:
    ImageConverter()
        : it_(nh_)
    {
        // Subscribe to input video feed and publish output video feed
        image_sub_ = it_.subscribe("/iris/usb_cam/image_raw", 1,
                                   &ImageConverter::imageCb, this);
    

        cv::namedWindow("oprgb");
    }

    ~ImageConverter()
    {
        cv::destroyWindow("oprgb");
    }

    void imageCb(const sensor_msgs::ImageConstPtr &msg)
    {
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
        }
        catch (cv_bridge::Exception &e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }



   if (tracker.getPose(cv_ptr->image, tVec, rVec) > 0)
        {

            tracker.correctedPose(rVec, tVec, ctVec);
            data_msg.linear.x = ctVec[0] / 100;
            data_msg.linear.y = ctVec[1] / 100;
            data_msg.linear.z = ctVec[2] / 100;
            data_msg.angular.x = rVec[0];
            data_msg.angular.y = rVec[1];
            data_msg.angular.z = rVec[2];

            bool_msg.data = 1;
            vishnu_cam_data_pub.publish(data_msg);
        }
        else
        {
            bool_msg.data = 0;
        }

            vishnu_cam_detection_pub.publish(bool_msg);

         // Update GUI Window
        cv::imshow("oprgb", cv_ptr->image);
        cv::waitKey(3);
    }

};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "vishnu_cam_node");
    ros::NodeHandle n;
    ImageConverter ic;

    // Publish to bodyframe pos of tag to topic /vishnu_cam_data
    vishnu_cam_data_pub = n.advertise<geometry_msgs::Twist>("vishnu_cam_data", 1);
    // Publish whether cam is detecting the ARtag
    vishnu_cam_detection_pub = n.advertise<std_msgs::Bool>("vishnu_cam_detection", 1);


    ros::spin();
    return 0;
}