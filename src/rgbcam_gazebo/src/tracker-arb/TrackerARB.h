//
//  TrackerAR.hpp
//  aruco_tracker
//
//  Created by Vishnu R Menon on 27/5/19.
//

#ifndef TrackerAR2_h
#define TrackerAR2_h

#include <opencv2/core.hpp>
#include <opencv2/aruco.hpp>

#include "../tracker-base/Tracker.h"
#include "../camera-calibration/CVCalibration.h"

using namespace std;
using namespace cv;

class TrackerARB : public Tracker {
  // TODO: Add these to the contructor
  float markerLength;
  float markerSeparation = 8.3f;
  int markersX = 2;
  int markersY = 2;
  bool showRejected = false;
  int inputwidth = 640;
  int inputheight = 480;
  const double pi = atan(1) * 4;
  double fovx, fovy;
  Ptr<aruco::Dictionary> markerDict = aruco::getPredefinedDictionary(aruco::PREDEFINED_DICTIONARY_NAME::DICT_4X4_50);
  vector<int> markerIds;
  vector<vector<Point2f>> markerCorners, rejectedCorners;
  Ptr<aruco::DetectorParameters> detectorParams = aruco::DetectorParameters::create();;
  Ptr<aruco::Board> board;
public:
  TrackerARB(CVCalibration& cvl, float markerLength);
  
  int getPose(Mat& frame, Vec3d& translationVec, Vec3d& rotationVec) override;
};

#endif /* TrackerAR2_h */
