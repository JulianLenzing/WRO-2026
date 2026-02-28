#ifndef LINE_QUALITY_SLAM_H
#define LINE_QUALITY_SLAM_H

#include <iostream>
#include <vector>
#include <random>

#include "Vec2f.h"
#include "Line.h"
#include "LidarPoint.h"
#include "DisplayData.h"

using namespace std;

struct intersectionIndexPair {
    int index{-1};
    Vec2f point;
};

void generateLandmarks(vector<Line>& lms);

void generateTestPoints(vector<LidarPoint>& lidarPoints, const Vec2f& pos, const vector<Line>& lms,
    const float& angleNoiseStdDeg = 2.0f, const float& distanceNoiseStd = 0.2f, const int& rayCount = 50);

bool isPointUseable(const Vec2f& xRange, const Vec2f& yRange, const Vec2f& dir, const vector<Line>& lms, size_t& landmark);

optional<Vec2f> processLidarPoints(const vector<LidarPoint>& lidarPoints, const vector<Line>& lms, Vec2f estimatedPosition, Vec2f xRange, Vec2f yRange);

#endif //LINE_QUALITY_SLAM_H