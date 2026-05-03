#pragma once

#include <chrono>
#include <cmath>
#include <algorithm>

#include "RobotSystem.h"
#include "DisplayData.h"
#include "Slam.h"

//#define USE_ENCODER_FOR_HEADING

void updateGyro(RobotSystem& robot);
void updateEncoder(RobotSystem& robot);
void updateLidar(RobotSystem& robot, std::chrono::milliseconds lidarDt);
void updateCamera(RobotSystem& robot);
