#pragma once

#include <chrono>

#include "State.h"
#include "RobotSystem.h"
#include "../RobotSystem.h"
#include "slam.h"
#include "../slam.h"

class StartState : public State{
	void enter(RobotSystem& robot) override 
	{
		robot.gpioController.setLed1High();
	}

    void update(RobotSystem& robot) override
    {
		if(robot.gpioController.queryButton()) robot.startActivated = true;
		robot.displayUI.update();
		std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    
    void exit(RobotSystem& robot) override 
    {
		robot.gpioController.setLed2High();
		robot.startTime = std::chrono::high_resolution_clock::now();

		robot.position = Vec2f(1.5f, 0.5f);
		robot.heading = 0.0f;

		// Start Lidar
		startLidar(robot.lidarDriver);

		// Init other sensors
		robot.encoderController.reset();
		robot.gyro.reset();
		
		// Start guidance
		robot.guidanceData.setRobotData(robot.position, robot.heading);
		robot.guidanceData.start();

		// Determine run direction
		LidarScan lidarScan;
		do
		{
			getLidarScan(robot.lidarDriver, lidarScan, 1, 0.25);
		}
		while (!getRunDirection(robot.position, robot.heading, lidarScan, robot.runDirection));
		if (robot.runDirection == RUN_DIRECTION_CCW) robot.heading = 0;
		else robot.heading = M_PI;
		robot.pathfinder.setRunDirection(robot.runDirection);
	}
	
	std::string name() const override {return "StartState";}
};
