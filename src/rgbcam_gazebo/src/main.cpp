#include <opencv2/core.hpp>

#include "tracker-ar/TrackerAR.h"

using namespace cv;
using namespace std;

int main(int argc, char **argv) {
    const auto arucoSquareDimension = 3.70f;
    CVCalibration cvl("CalibParams.txt");
    TrackerAR tracker(cvl, arucoSquareDimension);
    
    int port = 0;
    if (argc>1) port = stoi(argv[1]);
    
    tracker.startStreamingTrack(port);
    return 0;
}
