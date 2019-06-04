#include "Tracker.h"

#include <iostream>
#include <iomanip>

#include <opencv2/aruco.hpp>


using namespace std;
using namespace cv;

Tracker::Tracker(CVCalibration &cvl) {
  cameraMatrix = cvl.cameraMatrix;
  distCoeffs = cvl.distCoeffs;
}

int CLOCK() {
  struct timespec t{};
  clock_gettime(CLOCK_MONOTONIC, &t);
  return static_cast<int>((t.tv_sec * 1000) + (t.tv_nsec * 1e-6));
}

double _avgdur = 0;
int _fpsstart = 0;
double _avgfps = 0;
double _fps1sec = 0;

double avgDur(double newdur) {
  _avgdur = 0.98 * _avgdur + 0.02 * newdur;
  return _avgdur;
}

double avgFPS() {
  if (CLOCK() - _fpsstart > 1000) {
    _fpsstart = CLOCK();
    _avgfps = 0.7 * _avgfps + 0.3 * _fps1sec;
    _fps1sec = 0;
  }
  
  _fps1sec++;
  return _avgfps;
}

template<typename T>
void printE(T t, const int &width) {
  cout << left << setw(width) << setfill(' ') << t;
}

void Tracker::loopedTracking(VideoCapture vid) {
  Mat frame;
  Vec3d rotationVec, translationVec;
  int frameno = 0;
  
  cout << "FrameNo\t\tTimestamp\t\t\t\t\tRunningTime\t\tFPS\t\tDist1\t\tDist2\t\tDist3\n";
  namedWindow("Camera Feed", WINDOW_AUTOSIZE);
  while (true) {
    if (!vid.read(frame)) {
      cerr << "Unable to read next frame. Ending tracking.\n";
      break;
    };
    
    auto start = static_cast<clock_t>(CLOCK());
    getPose(frame, translationVec, rotationVec);
    if (getPose(frame, translationVec, rotationVec) == 1) {
      aruco::drawAxis(frame, cameraMatrix, distCoeffs, rotationVec, translationVec, 0.08f);
      double dur = CLOCK() - start;
      auto t = time(nullptr);
      auto tm = *localtime(&t);
      frameno++;
      cout << frameno << "\t\t"
           << put_time(&tm, "%d/%m %H:%M:%S") << "\t\t\t\t\t"
           << avgDur(dur) << "\t\t"
           << avgFPS() << "\t\t"
           << translationVec[0] << "\t\t"
           << translationVec[1] << "\t\t"
           << translationVec[2] << "\t\t"
           << endl;
    }
    imshow("Camera Feed", frame);
    if (waitKey(60) >= 0) break;
  }
}

bool Tracker::startStreamingTrack(int port) {
  VideoCapture vid(port);
  if (!vid.isOpened()) {
    cerr << "Unable to read video stream. Is the camera mount path correct?\n";
    return false;
  }
  loopedTracking(vid);
  return true;
}

bool Tracker::startVideoTrack(const string &fname) {
  VideoCapture vid(fname);
  if (!vid.isOpened()) {
    cerr << "Unable to read video file. Is the filepath correct?\n";
    return false;
  }
  loopedTracking(vid);
  return true;
}

