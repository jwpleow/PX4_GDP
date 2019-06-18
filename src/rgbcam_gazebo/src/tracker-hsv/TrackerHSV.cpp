#include "TrackerHSV.h"

#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <iostream>
#include <numeric>
#include <algorithm>

using namespace std;
using namespace cv;


TrackerHSV::TrackerHSV(CVCalibration &cvl, bool showFrame)
    : Tracker(cvl, showFrame) {
  getObjectPoints(1470.f, 1782.f);
}

int TrackerHSV::getPose(Mat &frame, Vec3d &tVec, Vec3d &rVec) {
  int detectedBoard = 0;
//  cerr << "Detected pad" << endl;

  
  solvePnP(objectCorners, markerCorners, cameraMatrix, distCoeffs, rVec, tVec);
  drawFrameAxes(frame, cameraMatrix, distCoeffs, rVec, tVec, 5);
//  cerr << tVec << endl;
  return 1;
}

bool TrackerHSV::detectLandingPad(Mat &frame) {
  Mat frameHSV, thresh, edges;
  Mat tmp = Mat::zeros(frame.size(), frame.type());
  cvtColor(frame, frameHSV, COLOR_BGR2HSV);
  
  inRange(frameHSV, Scalar(0, 0, 0), Scalar(0, 0, 255), thresh);
  
  int strength = 7;
  erode(thresh, thresh, getStructuringElement(MORPH_ELLIPSE, Size(strength, strength)));
  dilate(thresh, thresh, getStructuringElement(MORPH_ELLIPSE, Size(strength, strength)));
  
  dilate(thresh, thresh, getStructuringElement(MORPH_ELLIPSE, Size(2 * strength, 2 * strength)));
  erode(thresh, thresh, getStructuringElement(MORPH_ELLIPSE, Size(2 * strength, 2 * strength)));
  imshow("binary", thresh);
  GaussianBlur(thresh, thresh, Size(7, 7), 2.0, 2.0);
  Canny(thresh, edges, 66.0, 133.0, 3);
  
  vector<Vec2f> lines;
  HoughLines(edges, lines, 1, CV_PI / 180, 50, 0, 0);
  
  ostringstream str;
  str << "Detected " << lines.size() << " lines." << endl;
  putText(edges, str.str(),
          Point(20, 440), // Coordinates
          FONT_HERSHEY_COMPLEX_SMALL, // Font
          1.0, // Scale. 2.0 = 2x bigger
          Scalar(200, 100, 100), // BGR Color
          1, // Line Thickness (Optional)
          LINE_AA); // Anti-alias (Optional)
  imshow("Edges", edges); //show the thresholded image
  Mat linee = frame.clone();
  for (size_t i = 0; i < lines.size(); i++) {
    float rho = lines[i][0], theta = lines[i][1];
    Point pt1, pt2;
    double a = cos(theta), b = sin(theta);
    double x0 = a * rho, y0 = b * rho;
    pt1.x = cvRound(x0 + 1000 * (-b));
    pt1.y = cvRound(y0 + 1000 * (a));
    pt2.x = cvRound(x0 - 1000 * (-b));
    pt2.y = cvRound(y0 - 1000 * (a));
    line(linee, pt1, pt2, Scalar(0, 255, 0), 3, LINE_AA);
  }
  imshow("hough", linee);
  
  
  vector<Point2f> intersections;
  for (size_t i = 0; i < lines.size(); i++) {
    for (size_t j = 0; j < lines.size(); j++) {
      Vec2f line1 = lines[i];
      Vec2f line2 = lines[j];
      if (acceptLinePair(line1, line2, CV_PI / 32)) {
        Point2f intersection = computeIntersect(line1, line2);
        intersections.push_back(intersection);
      }
    }
    
  }
  
  if (!intersections.empty()) {
    vector<Point2f>::iterator i;
    for (i = intersections.begin(); i != intersections.end(); ++i) {
//      cerr << "Intersection is " << i->x << ", " << i->y << endl;
      circle(tmp, *i, 1, Scalar(0, 0, 255), 6);
    }
//    cout << "Next" << endl;
  }
  
  dilate(tmp, tmp, getStructuringElement(MORPH_ELLIPSE, Size(5 * strength, 5 * strength)));
  
  Mat tmp_canny;
  Canny(tmp, tmp_canny, 50, 150, 3);
  vector<Vec4i> hierarchy;
  vector<vector<Point>> contours;
  findContours(tmp_canny, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE, Point(0, 0));
  imshow("tmp_canny", tmp_canny);
  if (contours.size() != 8) return false;
// get the moments
  vector<Moments> mu(contours.size() / 2);
  for (int i = 0; i < contours.size(); i += 2) {
    mu[i / 2] = moments(contours[i], false);
  }

// get the centroid of figures.
  markerCorners.resize(mu.size());
  for (int i = 0; i < mu.size(); i++) {
    markerCorners[i] = Point2f(mu[i].m10 / mu[i].m00, mu[i].m01 / mu[i].m00);
    putText(frame, to_string(i),
            markerCorners[i], // Coordinates
            FONT_HERSHEY_COMPLEX_SMALL, // Font
            1.0, // Scale. 2.0 = 2x bigger
            Scalar(200, 0, 200), // BGR Color
            1, // Line Thickness (Optional)
            LINE_AA); // Anti-alias (Optional)
  }
  
  imshow("tmp_canny2", tmp_canny);
  return true;
}


bool TrackerHSV::acceptLinePair(Vec2f line1, Vec2f line2, double minTheta) {
  float theta1 = line1[1], theta2 = line2[1];
  
  if (theta1 < minTheta) {
    theta1 += CV_PI; // dealing with 0 and 180 ambiguities...
  }
  
  if (theta2 < minTheta) {
    theta2 += CV_PI; // dealing with 0 and 180 ambiguities...
  }
  
  return abs(theta1 - theta2) > minTheta;
}

// the long nasty wikipedia line-intersection equation...bleh...
Point2f TrackerHSV::computeIntersect(Vec2f line1, Vec2f line2) {
  vector<Point2f> p1 = lineToPointPair(line1);
  vector<Point2f> p2 = lineToPointPair(line2);
  
  float denom = (p1[0].x - p1[1].x) * (p2[0].y - p2[1].y) - (p1[0].y - p1[1].y) * (p2[0].x - p2[1].x);
  Point2f intersect(((p1[0].x * p1[1].y - p1[0].y * p1[1].x) * (p2[0].x - p2[1].x) -
                     (p1[0].x - p1[1].x) * (p2[0].x * p2[1].y - p2[0].y * p2[1].x)) / denom,
                    ((p1[0].x * p1[1].y - p1[0].y * p1[1].x) * (p2[0].y - p2[1].y) -
                     (p1[0].y - p1[1].y) * (p2[0].x * p2[1].y - p2[0].y * p2[1].x)) / denom);
  
  return intersect;
}

vector<Point2f> TrackerHSV::lineToPointPair(Vec2f line) {
  vector<Point2f> points;
  
  float r = line[0], t = line[1];
  double cos_t = cos(t), sin_t = sin(t);
  double x0 = r * cos_t, y0 = r * sin_t;
  double alpha = 1000;
  
  points.emplace_back(x0 + alpha * (-sin_t), y0 + alpha * cos_t);
  points.emplace_back(x0 - alpha * (-sin_t), y0 - alpha * cos_t);
  
  return points;
}

void TrackerHSV::getObjectPoints(double markerLengthX, double markerLengthY) {
  CV_Assert(markerLengthX > 0 && markerLengthY > 0);
  
  objectCorners.resize(4);
  // set coordinate system in the middle of the marker, with Z pointing out
  objectCorners[0] = Vec3d(-markerLengthX / 2.f, markerLengthY / 2.f, 0);
  objectCorners[1] = Vec3d(markerLengthX / 2.f, markerLengthY / 2.f, 0);
  objectCorners[2] = Vec3d(markerLengthX / 2.f, -markerLengthY / 2.f, 0);
  objectCorners[3] = Vec3d(-markerLengthX / 2.f, -markerLengthY / 2.f, 0);
}

template<typename T>
vector<size_t> sort_indexes(const vector<T> &v) {
  // initialize original index locations
  vector<size_t> idx(v.size());
  iota(idx.begin(), idx.end(), 0);
  // sort indexes based on comparing values in v
  sort(idx.begin(), idx.end(),
       [&v](size_t i1, size_t i2) { return v[i1] < v[i2]; });
  return idx;
}

//void TrackerHSV::sortCorners(vector<Point2f> &_markerCorners, vector<Point2f> &_sortedCorners) {
//  _sortedCorners.resize(_markerCorners.size());
//  vector<double> distances(_markerCorners.size());
//  int i = -1;
//  for(auto &corner : _markerCorners) {
//    distances[++i] = norm(_markerCorners[0]-corner);
//  }
//  int j = -1;
//  for (auto k: sort_indexes(distances)) {
//    _sortedCorners[++j] = _markerCorners[k];
//  }
//}
