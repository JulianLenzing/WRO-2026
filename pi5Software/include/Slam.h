#pragma once

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
#include "Run_Type.h"

using namespace std;

struct intersectionIndexPair {
    int index{-1};
    Vec2f point;
};

class Slam
{
public:
    void generateTestPoints(vector<LidarPoint>& lidarPoints, const Vec2f& pos, const vector<Line>& lms,
        const float& angleNoiseStdDeg = 2.0f, const float& distanceNoiseStd = 0.2f, const int& rayCount = 50);

    int getUsablePoints(LidarScan scan, Vec2f estimatedPosition, const Environment& environment, LidarScan& useableScan);

    int getDistanceUseablePoints(const LidarScan& scan, LidarScan& useableScan);

    optional<float> lidarEstimateHeading(const LidarScan& scan, const Environment& environment, Vec2f estimatedPosition);

    optional<Vec2f> lidarEstimatePosition(const LidarScan& scan, const Environment& environment, const Vec2f& estimatedPosition);

    int getRunDirection(const Vec2f& position, const float& heading, const LidarScan& scan, enum RUN_TYPE runType, enum RUN_DIRECTION& runDirection);

    float minPointDistance = 0.15f;
    float maxPointDistance = 3.65f;
    float maxDeltaPosition = 0.2f;
    float maxDistanceDeviation = 0.25f;

    int minPointsForLine = 35;
    float maxLineDeviation = 0.349f; // Atmost pi/2

    float minimumPerpendicularDistanceForObstacleRunDirection = 0.6f;
    int minPointDifferenceForObstacleRunDirection = 5;
    int minPointsForObstacleRunDirection = 10;

    float minWallDistanceDifferenceForOpeningRunDirection = 0.15f;
    float angleForOpeningRunDirectionDetermination = 5.0f/180.0f*M_PI;

private:
    static float angleWeight(const Line& a, const Line& b);
    std::optional<Vec2f> weightedAngleAverageSegmentIntersections(const std::vector<Line>& lines);
    bool isPointDistanceUseable(const LidarPoint& lp, const float& minDistance, const float& maxDistance);
    bool isPointUseable(LidarPoint& lp, Vec2f estimatedPosition, float minDistance, float maxDistance, Environment environment);
    Line linearRegression(const vector<Vec2f>& points);
    optional<float> compareLines(const Line& a, const Line& b);
    LidarPoint vec2fToLidarPoint(const Vec2f& point);
};
