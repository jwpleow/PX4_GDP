#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>

#include <fstream>
#include <iostream>

#include "CVCalibration.h"

using namespace cv;
using namespace std;

int main(int argc, char **argv) {
  VideoCapture vid(1);
  
  if (!vid.isOpened()) return 0;
  
  namedWindow("Webcam Feed", WINDOW_AUTOSIZE);
  const auto chessboardTileSize = 0.0265f;
  const auto chessboardDimensions = Size(9, 6);
  CVCalibration c(chessboardDimensions, chessboardTileSize);
  
  c.startStreamingCalibration(vid, "Webcam Feed");
  cout << "Calibration has ended";
  if(c.isCalibrated()) {
    cout << " successfully" << endl;
    string filename = argc > 1 ? argv[1] : "CameraMatrix2.txt";
    if(c.saveCalibrationMatrices(filename)) {
      cout << "The camera parameters are saved in " << filename << endl;
    } else {
      cout << "The camera parameters could not be saved in " << filename << endl;
    }
  } else {
    cout << " unsuccessfully" << endl;
  }
  
  return 0;
}
