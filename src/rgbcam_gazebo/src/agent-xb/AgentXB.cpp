#include <utility>

//
// Created by Vishnu R Menon on 2019-06-15.
//

#include "AgentXB.h"
#include <iostream>
#include <iomanip>
#include "SMA.hpp"

using namespace std;



int AgentXB::CLOCK() {
  struct timespec t{};
  clock_gettime(CLOCK_MONOTONIC, &t);
  return static_cast<int>((t.tv_sec * 1000) + (t.tv_nsec * 1e-6));
}

double AgentXB::avgDur(double newdur) {
  _avgdur = 0.98 * _avgdur + 0.02 * newdur;
  return _avgdur;
}

double AgentXB::avgFPS() {
  if (CLOCK() - _fpsstart > 1000) {
    _fpsstart = CLOCK();
    _avgfps = 0.95 * _avgfps + 0.05 * _fps1sec;
    _fps1sec = 0;
  }
  
  _fps1sec++;
  return _avgfps;
}

int AgentXB::getFrameWidth() {
  return trackers[0]->frameWidth;
}

int AgentXB::getFrameHeight() {
  return trackers[0]->frameHeight;
}

AgentXB::AgentXB(int _mode, bool _showFrame) {
  mode = _mode;
  showFrame = _showFrame;
}

void AgentXB::addTracker(unique_ptr<Tracker> tracker) {
  trackers.emplace_back(move(tracker));
  size++;
}

bool AgentXB::startVideoTrack(const string &fname, bool saveVideo, string filename) {
  VideoCapture vid(fname);
  if (!vid.isOpened()) {
    cerr << "Unable to read video file. Is the filepath correct?\n";
    return false;
  }
  loopedTracking(vid, saveVideo, filename);
  return true;
}

void AgentXB::loopedTracking(VideoCapture vid, bool saveVideo, string filename) {
  Mat frame;
  VideoWriter rawVideo, procVideo;
  string rawFilename = filename + " (Raw).avi";
  string procFilename = filename + " (Proc).avi";
  Vec3d rVec, tVec;
  int datano = 0;
  
  cout << "Row\tFrame\tRunning\tTimestamp\tFPS\tDist1\tDist2\tDist3\n";
  if (showFrame) namedWindow("Camera Feed", WINDOW_AUTOSIZE);
  if (saveVideo) {
    rawVideo = VideoWriter(rawFilename, VideoWriter::fourcc('M', 'J', 'P', 'G'), 30,
                           Size(getFrameWidth(), getFrameHeight()),
                           true);
    procVideo = VideoWriter(procFilename, VideoWriter::fourcc('M', 'J', 'P', 'G'), 30,
                            Size(getFrameWidth(), getFrameHeight()),
                            true);
  }
  bool play = true;
  while (play) {
    if (!vid.read(frame)) {
      cerr << "Unable to read next frame. Ending tracking.\n";
      break;
    };
    if (saveVideo) rawVideo.write(frame);
    
    auto start = static_cast<clock_t>(CLOCK());
    
    if (greedyTrack(frame, tVec, rVec)) {
      smaPose(tVec, tVec);
      
      datano++;
      double dur = CLOCK() - start;
      auto t = time(nullptr);
      auto tm = *localtime(&t);
      cout << datano << "\t"
           << vid.get(CAP_PROP_POS_FRAMES) << "\t"
           //           << put_time(&tm, "%H:%M:%S") << "\t"
                      << avgDur(dur) << "\t"
                      << avgFPS() << "\t"
           << activeTracker << "\t"
           << tVec[0] << "\t"
           << tVec[1] << "\t"
           << tVec[2] << "\t"
           << endl;
    }
    if (showFrame) imshow("Camera Feed", frame);
    if (saveVideo) procVideo.write(frame);
    char character = static_cast<char>(waitKey(50));
    switch (character) {
      case ' ': // Space key -> save image
        waitKey(0);
      case 27: // Esc key -> exit prog.
        play = false;
      default:
        break;
    }
  }
  
  // When everything done, release the video capture and write object
  vid.release();
  
  if (saveVideo) {
    rawVideo.release();
    procVideo.release();
  }
  
  // Closes all the windows
  if (showFrame) destroyAllWindows();
}

bool AgentXB::greedyTrack(Mat &frame, Vec3d &tVec, Vec3d &rVec) {
  activeTracker = 0;
  for (auto &tracker : trackers) {
    activeTracker++;
    if (tracker->detectLandingPad(frame)) {
      if (tracker->getPose(frame, tVec, rVec) > 0) {
        return true;
      }
    }
  }
  return false;
}

const int SMAL = 5;
SMA ctSMA[3]{SMA(SMAL), SMA(SMAL), SMA(SMAL)};

void AgentXB::smaPose(const Vec3d &ctVec, Vec3d &sctVec) {
  for (int i = 0; i < 3; ++i) {
    ctSMA[i].add(ctVec[i]);
    sctVec[i] = ctSMA[i].avg();
  }
}