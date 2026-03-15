#pragma once

#include "StateMachine.h"
#include "State.h"
#include "lidar.h"
#include "poseEstimation.h"
#include "LidarPoint.h"
#include "Vec2f.h"
#include "Graphics.h"
#include "DisplayData.h"
#include "DisplayUserInterface.h"
#include "GuidanceData.h"
#include "guidance.h"
#include "ServoControl.h"
#include "sl_lidar_driver.h"
#include "GpioController.h"
#include "EncoderController.h"

class RobotSystem{
	public:
	// Software
	Graphics gp;
	Visibility visibility;
	DisplayUserInterface displayUI;
			
	GuidanceData guidanceData;
	Vec2f estimatedPosition;
	bool startActivated;

	// Actuators
	GpioController gpioController;
	ServoControl steeringServo;
	ServoControl motor;

	// Sensors
	sl::ILidarDriver* lidarDriver;
	//EncoderController encoderController;

	RobotSystem()
		: steeringServo(0,0,0),
		  motor(0,0,0),
		  gpioController(),
		  //encoderController(gpioController),
		  gp(1000,1000, BLACK),
		  displayUI(visibility),
		  lidarDriver(nullptr),
		  estimatedPosition(0.5f,0.5f),
		  startActivated(false)
	{}
};
