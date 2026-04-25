#pragma once

#include <chrono>

#include "lidar.h"
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
#include "Environment.h"
#include "Gyro.h"
#include "ObstacleDetection.h"
#include "Pathfinder.h"
#include "Camera.h"

class RobotSystem{
	public:
	// Software
	Graphics gp;
	Visibility visibility;
	DisplayUserInterface displayUI;
	std::thread guidanceThread;
	std::chrono::high_resolution_clock::time_point initTime;
	std::chrono::high_resolution_clock::time_point startTime;
	Environment environment;
	GuidanceData guidanceData;
	bool startActivated;
	bool stopRequested;
	ObstacleDetection obstacleDetection;
	Pathfinder pathfinder;
	enum RUN_DIRECTION runDirection;

	// Pose
	float heading;
	Vec2f position;

	// Actuators
	GpioController gpioController;

	// Sensors
	sl::ILidarDriver* lidarDriver;
	EncoderController encoderController;
	Gyro gyro;
	Camera camera;

	RobotSystem() :
		gpioController(),
		encoderController(gpioController),
		gyro(),
		camera(),
		gp(1000,1000, BLACK),
		displayUI(visibility),
		lidarDriver(nullptr),
		environment(),
		guidanceThread(guidanceMain, ref(guidanceData)),
		startActivated(false),
		stopRequested(false),
		initTime(std::chrono::high_resolution_clock::now()),
		startTime(std::chrono::high_resolution_clock::now()),
		heading(0.0f),
		position(0.0f, 0.0f),
		runDirection(RUN_DIRECTION_CCW),
		obstacleDetection(),
		pathfinder()
	{}
};
