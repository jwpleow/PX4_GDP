#include "TrackerARB.h"

#include <iostream>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>

using namespace std;
using namespace cv;


static bool readDetectorParameters(const string &fname, Ptr<aruco::DetectorParameters> &params) {
  FileStorage fs(fname, FileStorage::READ);
  if (!fs.isOpened())
    return false;
  fs["adaptiveThreshWinSizeMin"] >> params->adaptiveThreshWinSizeMin;
  fs["adaptiveThreshWinSizeMax"] >> params->adaptiveThreshWinSizeMax;
  fs["adaptiveThreshWinSizeStep"] >> params->adaptiveThreshWinSizeStep;
  fs["adaptiveThreshConstant"] >> params->adaptiveThreshConstant;
  fs["minMarkerPerimeterRate"] >> params->minMarkerPerimeterRate;
  fs["maxMarkerPerimeterRate"] >> params->maxMarkerPerimeterRate;
  fs["polygonalApproxAccuracyRate"] >> params->polygonalApproxAccuracyRate;
  fs["minCornerDistanceRate"] >> params->minCornerDistanceRate;
  fs["minDistanceToBorder"] >> params->minDistanceToBorder;
  fs["minMarkerDistanceRate"] >> params->minMarkerDistanceRate;
  fs["cornerRefinementMethod"] >> params->cornerRefinementMethod;
  fs["cornerRefinementWinSize"] >> params->cornerRefinementWinSize;
  fs["cornerRefinementMaxIterations"] >> params->cornerRefinementMaxIterations;
  fs["cornerRefinementMinAccuracy"] >> params->cornerRefinementMinAccuracy;
  fs["markerBorderBits"] >> params->markerBorderBits;
  fs["perspectiveRemovePixelPerCell"] >> params->perspectiveRemovePixelPerCell;
  fs["perspectiveRemoveIgnoredMarginPerCell"] >> params->perspectiveRemoveIgnoredMarginPerCell;
  fs["maxErroneousBitsInBorderRate"] >> params->maxErroneousBitsInBorderRate;
  fs["minOtsuStdDev"] >> params->minOtsuStdDev;
  fs["errorCorrectionRate"] >> params->errorCorrectionRate;
  return true;
}

TrackerARB::TrackerARB(CVCalibration &cvl, float _markerLength) : Tracker(cvl) {
  markerLength = _markerLength;
  Ptr<aruco::GridBoard> gridboard =
      aruco::GridBoard::create(markersX, markersY, markerLength, markerSeparation, markerDict);
//  vector<int> boardMarkerIds = {1, 3, 1, 2};
//  gridboard->ids = boardMarkerIds;
  board = gridboard.staticCast<aruco::Board>();
  fovx = 2 * atan(inputwidth / (2 * cameraMatrix.at<double>(0, 0))) * (180.0 / pi);
  fovy = 2 * atan(inputheight / (2 * cameraMatrix.at<double>(1, 1))) * (180.0 / pi);
  cout << "info:FoVx~" << fovx << ":FoVy~" << fovy << ":vWidth~" << inputwidth << ":vHeight~" << inputheight << endl;
  if (!readDetectorParameters("detector_params.yml", detectorParams)) {
    cerr << "Invalid detector parameters file" << endl;
  }
  detectorParams->cornerRefinementMethod = aruco::CORNER_REFINE_SUBPIX;
}

// Print the calculated distance at bottom of image
void
drawVectors(Mat &frame, Scalar color, int lineWidth, int vOffset, int MarkerId, double X, double Y, double Distance,
            double cX, double cY) {
  char cad[100];
  sprintf(cad, "Board:%i, Distance:%0.3fm, X:%0.3f, Y:%0.3f, cX:%0.3f, cY:%0.3f", MarkerId, Distance, X, Y, cX, cY);
  Point cent(10, vOffset);
  putText(frame, cad, cent, FONT_HERSHEY_SIMPLEX, std::max(0.5f, float(lineWidth) * 0.3f), color, lineWidth);
}

int TrackerARB::getPose(Mat &frame, Vec3d &tVec, Vec3d &rVec) {
  int detectedBoard = 0;
  vector<Vec3d> translationVec, rotationVec;
  if(showRejected) {
    aruco::detectMarkers(frame, markerDict, markerCorners, markerIds, detectorParams, rejectedCorners);
  } else {
    aruco::detectMarkers(frame, markerDict, markerCorners, markerIds);
  }
  
  if (!markerIds.empty()) {
    aruco::drawDetectedMarkers(frame, markerCorners, markerIds);
    detectedBoard = aruco::estimatePoseBoard(markerCorners, markerIds, board, cameraMatrix, distCoeffs, rVec, tVec);
//    for(int i = 0; i < 3; ++i) {
//      tVec[i] += correction[i];
//    }
  
    double distance = sqrt(pow(tVec[0], 2) + pow(tVec[1], 2) + pow(tVec[2], 2));
   
    if (detectedBoard > 0) {
//      aruco::drawAxis(frame, cameraMatrix, distCoeffs, rVec, tVec, 5);
      drawFrameAxes(frame, cameraMatrix, distCoeffs, rVec, tVec, 10);
    }
    if (showRejected && !rejectedCorners.empty()) {
        aruco::drawDetectedMarkers(frame, rejectedCorners, noArray(), Scalar(100, 0, 255));
    }
  }
  //    char key = (char) waitKey(5);
  
  
  //    aruco::estimatePoseSingleMarkers(markerCorners, arucoSquareDimension, cameraMatrix, distCoeffs, rotationVec,
  //                                   translationVec);
  //    if (!markerIds.empty() && markerIds[0] <= 4) {
  //      //    TODO: CHECK FOR MARKER ID MATCH
  //      tVec = translationVec[0];
  //      rVec = rotationVec[0];
  //      return 1;
  //    }
  
  return detectedBoard;
}

