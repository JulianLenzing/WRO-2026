#pragma once

#include <chrono>

#include "State.h"
#include "RobotSystem.h"
#include "../RobotSystem.h"

class StartState : public State{
	void enter(RobotSystem& robot) override 
	{
		robot.gpioController.setLed1High();
	}

    bool update(RobotSystem& robot) override
    {
		if(robot.gpioController.queryButton()) return true;
		robot.displayUI.update();
		std::this_thread::sleep_for(std::chrono::milliseconds(5));
		return false;
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
		std::vector<Obstacle> tmp;
		// Feed one frame before driving to avoid lag while driving; The list of obstacles is empty so no data can be messed up
		robot.obstacleDetection.feedImage(robot.camera.grabFrame(), tmp, Vec2f(0.0f, 0.0f), 0.0f);
		
		// Start guidance
		robot.guidanceData.setRobotData(robot.position, robot.heading);
		robot.guidanceData.start();
	}
	
	std::string name() const override {return "StartState";}
};
