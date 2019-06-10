#ifndef OBJECT_TRACKING_CAMERACALIBRATION_H
#define OBJECT_TRACKING_CAMERACALIBRATION_H

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>

using namespace cv;
using namespace std;

extern int DEFAULT_FRAME_WIDTH;
extern int DEFAULT_FRAME_HEIGHT;
extern double DEFAULT_DIST_COEFFS[5];
extern double DEFAULT_CAMERA_MATRIX[9];

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
  Mat distCoeffs = Mat(1, 5, CV_64F, DEFAULT_DIST_COEFFS);
  Mat cameraMatrix = Mat(3, 3, CV_64F, DEFAULT_CAMERA_MATRIX);
  int frameWidth = DEFAULT_FRAME_WIDTH;
  int frameHeight = DEFAULT_FRAME_HEIGHT;
  
  bool startStreamingCalibration(VideoCapture vid, string window);
  bool isCalibrated();
  bool saveCalibrationMatrices(string fname);
  bool loadCalibrationMatrices(string fname);
};

#endif