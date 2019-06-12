#ifndef ARUCO_TRACKING_TRACKER_H
#define ARUCO_TRACKING_TRACKER_H

#include <opencv2/core.hpp>
#include "../camera-calibration/CVCalibration.h"

using namespace std;
using namespace cv;

class Tracker {
  void loopedTracking(VideoCapture vid);
protected:
  Mat cameraMatrix, distCoeffs;
  int frameWidth, frameHeight;
  const double pi = atan(1) * 4;
  double fovx, fovy;
  bool showFrame;
public:
  explicit Tracker(CVCalibration& cvl, bool showFrame=true);
  virtual bool detectLandingPad(Mat& frame) = 0;
  virtual int getPose(Mat& frame, Vec3d& tVec, Vec3d& rVec) = 0;
  bool startStreamingTrack(int port = 0);
  bool startVideoTrack(const string& fname);
  
  void getOffsetPose(const Vec3d &rVec, const Vec3d &tVec, Vec3d &otVec);
  void getGlobalPose(const Vec3d &rVec, const Vec3d &tVec, Vec3d &ctVec) const;
  void smaPose(const Vec3d &ctVec, Vec3d &sctVec);
};

#endif //ARUCO_TRACKING_TRACKER_H
