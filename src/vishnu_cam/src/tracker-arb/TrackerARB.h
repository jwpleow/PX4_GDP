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
  bool showRejected = false;
  Ptr<aruco::Dictionary> markerDict = aruco::getPredefinedDictionary(aruco::PREDEFINED_DICTIONARY_NAME::DICT_4X4_50);
  vector<int> markerIds;
  vector<vector<Point2f>> markerCorners, rejectedCorners;
  Ptr<aruco::DetectorParameters> detectorParams = aruco::DetectorParameters::create();;
  Ptr<aruco::Board> board;
public:
  TrackerARB(CVCalibration& cvl, float markerLength, float markerSeparation, int markersX, int markersY, bool showFrame=true);
  
  int getPose(Mat& frame, Vec3d& translationVec, Vec3d& rotationVec) override;
};

#endif /* TrackerAR2_h */
