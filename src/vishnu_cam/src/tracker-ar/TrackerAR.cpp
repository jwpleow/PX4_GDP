#include "TrackerAR.h"

#include <iostream>

using namespace std;
using namespace cv;

TrackerAR::TrackerAR(CVCalibration& cvl, float _arucoSquareDimension) : Tracker(cvl) {
  arucoSquareDimension = _arucoSquareDimension;
}

int TrackerAR::getPose(Mat& frame, Vec3d& tVec, Vec3d& rVec) {
//  namedWindow("Camera Feed", WINDOW_AUTOSIZE);
  vector<Vec3d> translationVec, rotationVec;
  aruco::detectMarkers(frame, markerDict, markerCorners, markerIds);
  aruco::estimatePoseSingleMarkers(markerCorners, arucoSquareDimension, cameraMatrix, distCoeffs, rotationVec,
                                   translationVec);
  if(!markerIds.empty() && markerIds[0] <= 4) {
    //    TODO: CHECK FOR MARKER ID MATCH
    tVec = translationVec[0];
    rVec = rotationVec[0];
    return 1;
  }
//  for (int i = 0; i < markerIds.size(); i++) {
//    aruco::drawAxis(frame, cameraMatrix, distCoeffs, rotationVec[i], translationVec[i], 0.08f);
//  }
//  imshow("Camera Feed", frame);
  return 0;
}
