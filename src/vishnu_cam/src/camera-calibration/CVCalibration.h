#ifndef OBJECT_TRACKING_CAMERACALIBRATION_H
#define OBJECT_TRACKING_CAMERACALIBRATION_H

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>

using namespace cv;
using namespace std;

class CVCalibration {
  Size chessboardDimensions;
  float chessboardTileSize;
  int framesPerSecond = 20;
  bool calibrated = false;
  vector<Mat> calibrationImages;
  vector<vector<Point2f>> allFoundCorners;

  void createKnownBoardPosition(vector<Point3f> &corners);
  void getCornersFromImages(bool showResults = false);
  void getCameraMatrices();
  
public:
  explicit CVCalibration(string fname);
  CVCalibration(Size chessboardDimensions, float chessboardTileSize);
  
  Mat distCoeffs;
  Mat cameraMatrix = Mat::eye(3, 3, CV_64F);
  
  bool startStreamingCalibration(VideoCapture vid, string window);
  bool isCalibrated();
  bool saveCalibrationMatrices(string fname);
  bool loadCalibrationMatrices(string fname);
};

#endif
