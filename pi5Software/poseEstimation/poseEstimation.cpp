/* Standard */
#include <iostream>
#include <vector>
#include <optional>

/* Base classes */
#include "Line.h"
#include "Vec2f.h"
#include "LidarPoint.h"

/* Graphics */
#include "Graphics.h"
#include "DisplayData.h"

/* User interface */
#include "Config.h"
#include "ConfigFileEditor.h"

/* Slam */
#include "slam.h"

/* Functional / Helpers */
#include "Timer.h"

using namespace std;

// Persistent variables
vector<Line> lms;

void initPoseEstimation() {
    generateLandmarks(lms);
}

int doPoseEstimation(LidarScan scan, Vec2f estimatedPosition, float maxDelta, Vec2f& newEstimatedPosition) {    
    ConfigFileEditor cf("../");
    cf.readFile("config.txt");
    Config config(cf.getEntries());

    //cout<<"Estimated Position - X: "<<estimatedPosition.x<< " Y: " << estimatedPosition.y;
    dpd.appendPoint(estimatedPosition, RED, ESTIMATED_POSITION_POINT);
    dpd.appendLines(lms, WHITE, LANDMARK_LINE);
    vector<LidarPoint> lidarPoints = scan.scan;

    // Process points
    Vec2f xRange(estimatedPosition.x + maxDelta, estimatedPosition.x - maxDelta);
    Vec2f yRange(estimatedPosition.y + maxDelta, estimatedPosition.y - maxDelta);
    auto maybeNewEstimatedPosition = processLidarPoints(lidarPoints, lms, estimatedPosition, xRange, yRange);
    
    if (maybeNewEstimatedPosition.has_value()) {
        newEstimatedPosition = maybeNewEstimatedPosition.value();
        //cout<<"  ---  New estimated position - X: "<< newEstimatedPosition.x<< " Y: " << newEstimatedPosition.y<<endl;
        dpd.appendPoint(newEstimatedPosition, YELLOW, NEW_ESTIMATED_POSITION_POINT);
        return 1;
    }
    //else {cout<<"  ---  No new estimated position found"<<endl;}
    return 0;
}
