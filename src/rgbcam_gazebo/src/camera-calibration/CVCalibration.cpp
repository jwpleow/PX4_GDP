#include "CVCalibration.h"

#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>

#include <fstream>
#include <iostream>

using namespace cv;
using namespace std;

int DEFAULT_FRAME_WIDTH = 640;
int DEFAULT_FRAME_HEIGHT = 480;
double DEFAULT_DIST_COEFFS[5] = {-0.442373, 0.347708, 0.000532287, 0.0078344, -0.263354};
double DEFAULT_CAMERA_MATRIX[9] = {698.934, 0, 275.679, 0, 699.571, 249.775, 0, 0, 1};

CVCalibration::CVCalibration(Size _chessboardDimensions,
                             float _chessboardTileSize) {
  chessboardDimensions = _chessboardDimensions;
  chessboardTileSize = _chessboardTileSize;
}

void CVCalibration::createKnownBoardPosition(vector<Point3f> &corners) {
  for (int r = 0; r < chessboardDimensions.height; r++) {
    for (int c = 0; c < chessboardDimensions.width; c++) {
      corners.emplace_back(c * chessboardTileSize, r * chessboardTileSize,
                           0.0f);
    }
  }
}

void CVCalibration::getCornersFromImages(bool showResults) {
  for (auto iter = calibrationImages.begin(); iter != calibrationImages.end();
       iter++) {
    vector<Point2f> pointBuf;
    bool found = findChessboardCorners(*iter, chessboardDimensions, pointBuf,
                                       CALIB_CB_ADAPTIVE_THRESH |
                                       CALIB_CB_NORMALIZE_IMAGE);
    if (found) {
      allFoundCorners.push_back(pointBuf);
    }
    if (showResults) {
      drawChessboardCorners(*iter, chessboardDimensions, pointBuf, found);
      imshow("Chessboard Corners", *iter);
      waitKey(0);
    }
  }
}

void CVCalibration::getCameraMatrices() {
  vector<vector<Point2f>> chessboardImageSpacePoints;
  getCornersFromImages();
  
  vector<vector<Point3f>> worldSpaceCornerPoints(1);
  
  createKnownBoardPosition(worldSpaceCornerPoints[0]);
  worldSpaceCornerPoints.resize(chessboardImageSpacePoints.size(),
                                worldSpaceCornerPoints[0]);
  
  vector<Mat> rVectors, tVectors;
  distCoeffs = Mat::zeros(8, 1, CV_64F);
  
  calibrateCamera(worldSpaceCornerPoints, chessboardImageSpacePoints,
                  chessboardDimensions, cameraMatrix, distCoeffs, rVectors,
                  tVectors);
  calibrated = true;
}

bool CVCalibration::startStreamingCalibration(VideoCapture vid, string window) {
  Mat frame, drawToFrame;
  while (true) {
    if (!vid.read(frame))
      break;
    
    vector<Vec2f> foundPoints;
    bool found = findChessboardCorners(frame, chessboardDimensions, foundPoints,
                                       CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_NORMALIZE_IMAGE);
    frame.copyTo(drawToFrame);
    drawChessboardCorners(drawToFrame, chessboardDimensions, foundPoints,
                          found);
    imshow(window, found ? drawToFrame : frame);
    char character = static_cast<char>(waitKey(1000 / framesPerSecond));
    
    switch (character) {
      case ' ': // Space key -> save image
        if (found) {
          Mat temp;
          frame.copyTo(temp);
          calibrationImages.push_back(temp);
          cout << "Found Chessboard; Calibration Images Count: "
          << calibrationImages.size() << endl;
        }
      case 13: // Return key  -> run calibration
        if (calibrationImages.size() > 20) {
          getCameraMatrices();
        }
        break;
      case 27: // Esc key -> exit prog.
        return 0;
      default:
        break;
    }
  }
  return 1;
}

bool CVCalibration::isCalibrated() {
  return calibrated;
}

bool CVCalibration::saveCalibrationMatrices(string fname) {
  ofstream calibrationFile(fname);
  if (calibrationFile) {
    for (int r = 0; r < cameraMatrix.rows; r++) {
      for (int c = 0; c < cameraMatrix.cols; c++) {
        auto val = cameraMatrix.at<double>(r, c);
        calibrationFile << val << endl;
      }
    }
    
    for (int r = 0; r < distCoeffs.rows; r++) {
      for (int c = 0; c < distCoeffs.cols; c++) {
        auto val = distCoeffs.at<double>(r, c);
        calibrationFile << val << endl;
      }
    }
  }
  calibrationFile.close();
  return true;
}

bool CVCalibration::loadCalibrationMatrices(string fname) {
  ifstream calibrationFile(fname);
  string line;
  
  if (calibrationFile.is_open()) {
    getline(calibrationFile, line);
    frameWidth = stoi(line);
  
    getline(calibrationFile, line);
    frameHeight = stoi(line);
  
    for (int r = 0; r < cameraMatrix.rows; r++) {
      for (int c = 0; c < cameraMatrix.cols; c++) {
        getline(calibrationFile, line);
        cameraMatrix.at<double>(r, c) = stod(line);
      }
    }
    for (int r = 0; r < distCoeffs.rows; r++) {
      for (int c = 0; c < distCoeffs.cols; c++) {
        getline(calibrationFile, line);
        distCoeffs.at<double>(r, c) = stod(line);
      }
    }
    calibrationFile.close();
    return true;
  }
  return false;
}

CVCalibration::CVCalibration(string fname) {
  if (!loadCalibrationMatrices(fname)) {
    cerr << "Invalid calibration file, using default params for ELP_CAM (640x480)\n";
    cerr.flush();
  }
}
