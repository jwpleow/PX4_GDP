#ifndef ARUCO_TRACKING_TRACKER_H
#define ARUCO_TRACKING_TRACKER_H

#include <opencv2/core.hpp>
#include "../camera-calibration/CVCalibration.h"

using namespace std;
using namespace cv;

class Tracker {
  void loopedTracking(VideoCapture vid, bool saveVideo = false, string filename = "");

protected:
  Mat cameraMatrix, distCoeffs;
  const double pi = atan(1) * 4;
  double fovx, fovy;
  bool showFrame;
  double _avgdur = 0;
  int _fpsstart = 0;
  double _avgfps = 0;
  double _fps1sec = 0;
  
  int CLOCK();
  
  double avgDur(double newdur);
  
  double avgFPS();

public:
  int frameWidth, frameHeight;
  
  explicit Tracker(CVCalibration &cvl, bool showFrame = true);
  
  virtual bool detectLandingPad(Mat &frame) = 0;
  
  virtual int getPose(Mat &frame, Vec3d &tVec, Vec3d &rVec) = 0;
  
  bool startStreamingTrack(int port = 0, bool saveVideo = false, string filename = "TestVideo");
  
  bool startVideoTrack(const string &fname, bool saveVideo = false, string filename = "TestVideo");
  
  void getGlobalPose(const Vec3d &rVec, const Vec3d &tVec, Vec3d &ctVec) const;
};

#endif //ARUCO_TRACKING_TRACKER_H
