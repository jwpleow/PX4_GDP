#include <ros/ros.h>
#include <opencv2/core.hpp>
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


#define DEFAULT_PORT 0
#define VID_CAPTURE_WIDTH 640


cv::Mat frame;
cv::Vec3d tVec, rVec, ctVec, sctVec;

const float markerLength = 3.62;
const float markerSeparation = 2.63;
const int markersX = 6;
const int markersY = 8;
CVCalibration cvl("CalibParams.txt");
TrackerARB tracker(cvl, markerLength, markerSeparation, markersX, markersY, false);



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


        if (tracker.detectLandingPad(cv_ptr->image))
        {
            if (tracker.getPose(cv_ptr->image, tVec, rVec) > 0)
            {
                tracker.getOffsetPose(rVec, tVec, ctVec);
                tracker.smaPose(ctVec, sctVec);
                ROS_INFO("X: %f, Y: %f, Z: %f", sctVec[0], sctVec[1], sctVec[2]);
                data_msg.linear.x   = (float) (sctVec[0] /  100);
                data_msg.linear.y   = (float) (sctVec[1] / 100);
                data_msg.linear.z   = (float) (sctVec[2] / 100);
                data_msg.angular.x  = (float) rVec[0];
                data_msg.angular.y  = (float) rVec[1];
                data_msg.angular.z  = (float) rVec[2];
                vishnu_cam_data_pub.publish(data_msg);
                bool_msg.data = 1;

            }
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