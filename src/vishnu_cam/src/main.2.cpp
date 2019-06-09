#include <opencv2/core.hpp>
#include "tracker-arb/TrackerARB.h"

#define DEFAULT_PORT 0

using namespace cv;
using namespace std;

int main(int argc, char **argv) {
    const float markerLength = 3.70;
    const float markerSeparation = 8.70;
    const int markersXY = 2;
    CVCalibration cvl("CalibParams.txt");
    TrackerARB tracker(cvl, markerLength, markerSeparation, markersXY, true);
    
    int port = argc > 1 ? stoi(argv[1]) : DEFAULT_PORT;
    
    tracker.startStreamingTrack(port);
    return 0;
}
