#pragma once

/* Standard */
#include <iostream>
#include <vector>
#include <optional>

/* Base classes */
#include "Line.h"
#include "Vec2f.h"
#include "LidarPoint.h"
#include "Landmarks.h"

/* Graphics */
#include "DisplayData.h"

/* Slam */
#include "slam.h"

/* Functional / Helpers */
#include "Timer.h"

using namespace std;

#define MAX_DELTA_POSITION 0.2f

class PoseEstimator {
public:
    int update(LidarScan scan, Landmarks landmarks, Vec2f estimatedPosition, Vec2f& newEstimatedPosition) {    
        Vec2f xRange(estimatedPosition.x + MAX_DELTA_POSITION, estimatedPosition.x - MAX_DELTA_POSITION);
        Vec2f yRange(estimatedPosition.y + MAX_DELTA_POSITION, estimatedPosition.y - MAX_DELTA_POSITION);
        LidarScan useableScan;
        getUsablePoints(scan, xRange, yRange, landmarks, useableScan);
        for(const auto& lp : scan.scan) {dpd.appendPoint(lp.point() + estimatedPosition, GRAY, UNSUEABLE_LIDAR_POINT_POINT);}
        for(const auto& lp : useableScan.scan) {dpd.appendPoint(lp.point() + estimatedPosition, BLUE, USEABLE_LIDAR_POINT_POINT);}

        auto maybeNewEstimatedPosition = lidarEstimatePosition(useableScan, landmarks, estimatedPosition);

        if (maybeNewEstimatedPosition.has_value()) {
            newEstimatedPosition = maybeNewEstimatedPosition.value();
            //cout<<"  ---  New estimated position - X: "<< newEstimatedPosition.x<< " Y: " << newEstimatedPosition.y<<endl;
            return 1;
        }
        //else {cout<<"  ---  No new estimated position found"<<endl;}
        return 0;
    }

    Vec2f estimatedPosition;
};