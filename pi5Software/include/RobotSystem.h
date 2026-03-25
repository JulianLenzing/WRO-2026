#pragma once

#include <chrono>

#include "lidar.h"
#include "PoseEstimator.h"
#include "LidarPoint.h"
#include "Vec2f.h"
#include "Graphics.h"
#include "DisplayData.h"
#include "DisplayUserInterface.h"
#include "GuidanceData.h"
#include "guidance.h"
#include "sl_lidar_driver.h"
#include "GpioController.h"
#include "EncoderController.h"
#include "Landmarks.h"

class RobotSystem{
	public:
	// Software
	Graphics gp;
	Visibility visibility;
	DisplayUserInterface displayUI;
	std::thread guidanceThread;
	std::chrono::high_resolution_clock::time_point initTime;
	std::chrono::high_resolution_clock::time_point startTime;
	Landmarks landmarks;
	GuidanceData guidanceData;
	bool startActivated;

	// Pose
	float heading;
	Vec2f position;

	// Actuators
	GpioController gpioController;

	// Sensors
	sl::ILidarDriver* lidarDriver;
	EncoderController encoderController;

	RobotSystem() :
		  gpioController(),
		  encoderController(gpioController),
		  gp(1000,1000, BLACK),
		  displayUI(visibility),
		  lidarDriver(nullptr),
		  landmarks(),
		  poseEstimator(),
		  guidanceThread(guidanceMain, ref(guidanceData)),
		  startActivated(false),
		  initTime(std::chrono::high_resolution_clock::now()),
		  startTime(std::chrono::high_resolution_clock::now()),
		  heading(0.0f),
		  position(0.0f, 0.0f)
	{}
};
