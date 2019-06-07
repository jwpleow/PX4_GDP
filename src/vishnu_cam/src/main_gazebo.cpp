#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "tracker-arb/TrackerARB.h"
///< ROS headers
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Bool.h>


namespace cv

static const std::string OPENCV_WINDOW = "Image window";

    Vec3d tVec, rVec;
    CVCalibration cvl("CalibParams.txt");
    TrackerARB tracker(cvl, arucoSquareDimension);

class ImageConverter {
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;

public:
  ImageConverter()
      : it_(nh_) {
    // Subscribe to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/iris/usb_cam/image_raw", 1,
                               &ImageConverter::imageCb, this);
    image_pub_ = it_.advertise("/image_converter/output_video", 1);
    
    cv::namedWindow(OPENCV_WINDOW);
  }
  
  ~ImageConverter() {
    cv::destroyWindow(OPENCV_WINDOW);
  }
  
  void imageCb(const sensor_msgs::ImageConstPtr &msg) {
    cv_bridge::CvImagePtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);a
    }
    catch (cv_bridge::Exception &e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    
    // Update GUI Window
    cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    cv::waitKey(3);
    
    // Get Pose Estimate
    const auto arucoSquareDimension = 3.70f;



    
    // Output modified video stream
    image_pub_.publish(cv_ptr->toImageMsg());

     if (tracker.getPose(cv_ptr->image, tVec, rVec)) {
      
      data_msg.linear.x = tVec[0];
      data_msg.linear.y = tVec[1];
      data_msg.linear.z = tVec[2];
      data_msg.angular.x = tVec[0];
      data_msg.angular.y = tVec[1];
      data_msg.angular.z = tVec[2];
      
      bool_msg.data = 1;
      vishnu_cam_detection_pub.publish(bool_msg);
      vishnu_cam_data_pub.publish(data_msg);
    }
    else{
      bool_msg.data = 0;
      vishnu_cam_detection_pub.publish(bool_msg);
    }


  }
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "vishnu_cam_node");
  ImageConverter ic;
  ros::NodeHandle n;
  ros::Rate rate(10);
  
  // Publish to topic /vishnu_cam_data
  ros::Publisher vishnu_cam_data_pub = n.advertise<geometry_msgs::Twist>("vishnu_cam_data", 10);
  // Publish whether cam is detecting the ARtag
  ros::Publisher vishnu_cam_detection_pub = n.advertise<std_msgs::Bool>("vishnu_cam_detection", 10);
  

  ros::spin();
  return 0;
}