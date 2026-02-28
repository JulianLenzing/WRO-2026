#ifndef POSEESTIMATION_H
#define POSEESTIMATION_H

#include "LidarPoint.h"
#include "Graphics.h"

void initPoseEstimation();
int doPoseEstimation(LidarScan scan, Vec2f estimatedPosition, float maxDelta, Vec2f& newEstimatedPosition);

#endif
