//
//  TrackerARB.hpp
//  tracker_arb
//
//  Created by Vishnu R Menon on 27/5/19.
//

#ifndef TrackerARB_h
#define TrackerARB_h

#include <opencv2/core.hpp>
#include <opencv2/aruco.hpp>

#include "../tracker-base/Tracker.h"
#include "../camera-calibration/CVCalibration.h"

using namespace std;
using namespace cv;

class TrackerARB : public Tracker {
  bool showRejected = false;
  double offsetX, offsetY;
  Ptr<aruco::Dictionary> markerDict;
  vector<int> markerIds;
  vector<vector<Point2f>> markerCorners, rejectedCorners;
  Ptr<aruco::DetectorParameters> detectorParams = aruco::DetectorParameters::create();;
  Ptr<aruco::Board> board;
  
  void offsetPose(const Vec3d &rVec, const Vec3d &tVec, Vec3d &otVec);

public:
  TrackerARB(CVCalibration &cvl, float markerLength, float markerSeparation, int markersX, int markersY,
             int markerDictId, bool showFrame = true);
  
  bool detectLandingPad(Mat &frame) override;
  
  int getRawPose(Mat &frame, Vec3d &translationVec, Vec3d &rotationVec);
  
  int getPose(Mat &frame, Vec3d &translationVec, Vec3d &rotationVec) override;
};

#endif /* TrackerARB_h */
