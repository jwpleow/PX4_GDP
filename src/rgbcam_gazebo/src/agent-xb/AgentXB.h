//
// Created by Vishnu R Menon on 2019-06-15.
//

#ifndef TRACKER_MONO_AGENTXB_HPP
#define TRACKER_MONO_AGENTXB_HPP

#include "../tracker-base/Tracker.h"

using namespace std;

class AgentXB {
  vector<unique_ptr<Tracker>> trackers;
  int activeTracker = 0;
  int mode;
  int size = 0;
  bool showFrame;
  double _avgdur = 0;
  int _fpsstart = 0;
  double _avgfps = 0;
  double _fps1sec = 0;
  
  int CLOCK();
  
  double avgDur(double newdur);
  
  double avgFPS();
  
  void loopedTracking(VideoCapture vid, bool saveVideo = false, string filename = "");
  
  int getFrameWidth();
  
  int getFrameHeight();
  

public:
  static const int MODE_GREEDY = 0;
  static const int MODE_ROLLING = 1;
  bool greedyTrack(Mat &frame, Vec3d &tVec, Vec3d &rVec);
  
  void addTracker(unique_ptr<Tracker> t);
  
  bool startVideoTrack(const string &fname, bool saveVideo = false, string filename = "TestVideo");
  
  void smaPose(const Vec3d &ctVec, Vec3d &sctVec);
  
  AgentXB(int mode, bool showFrame);
};


#endif //TRACKER_MONO_AGENTXB_HPP
