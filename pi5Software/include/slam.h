#ifndef LINE_QUALITY_SLAM_H
#define LINE_QUALITY_SLAM_H

#include <iostream>
#include <vector>
#include <random>

#include "Vec2f.h"
#include "Line.h"
#include "LidarPoint.h"
#include "DisplayData.h"
#include "LidarPoint.h"
#include "Environment.h"
#include "Pathfinder.h"

using namespace std;

struct intersectionIndexPair {
    int index{-1};
    Vec2f point;
};

void generateTestPoints(vector<LidarPoint>& lidarPoints, const Vec2f& pos, const vector<Line>& lms,
    const float& angleNoiseStdDeg = 2.0f, const float& distanceNoiseStd = 0.2f, const int& rayCount = 50);

int getUsablePoints(LidarScan scan, Vec2f estimatedPosition, const Environment& environment, LidarScan& useableScan);

int getDistanceUseablePoints(const LidarScan& scan, LidarScan& useableScan);

optional<float> lidarEstimateHeading(const LidarScan& scan, const Environment& environment, Vec2f estimatedPosition);

optional<Vec2f> lidarEstimatePosition(const LidarScan& scan, const Environment& environment, const Vec2f& estimatedPosition);

int getRunDirection(const Vec2f& position, const float& heading, const LidarScan& scan, enum RUN_DIRECTION& runDirection);

#endif //LINE_QUALITY_SLAM_H
