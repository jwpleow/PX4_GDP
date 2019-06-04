//
//  TrackerAR.hpp
//  aruco_tracker
//
//  Created by Vishnu R Menon on 27/5/19.
//

#ifndef TrackerAR_h
#define TrackerAR_h

#include <opencv2/core.hpp>
#include <opencv2/aruco.hpp>

#include "../tracker-base/Tracker.h"
#include "../camera-calibration/CVCalibration.h"

using namespace std;
using namespace cv;

class TrackerAR : public Tracker {
  float arucoSquareDimension;
  Ptr<aruco::Dictionary> markerDict = aruco::getPredefinedDictionary(aruco::PREDEFINED_DICTIONARY_NAME::DICT_4X4_50);
  vector<int> markerIds;
  vector<vector<Point2f>> markerCorners, rejectedCorners;
  aruco::DetectorParameters markerParams;
public:
  TrackerAR(CVCalibration& cvl, float arucoSquareDimension);
  int getPose(Mat& frame, Vec3d& translationVec, Vec3d& rotationVec) override;
};

#endif /* TrackerAR_h */
