#pragma once

#include <chrono>

#include "State.h"
#include "RobotSystem.h"

class StartState : public State{
    void update(RobotSystem& robot) override
    {
		if(robot.gpioController.queryButton()) robot.startActivated = true;
		robot.displayUI.update();
		std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    
    void exit(RobotSystem& robot) override 
    {
		robot.startTime = std::chrono::high_resolution_clock::now();

		// Start Lidar
		startLidar(robot.lidarDriver);

		// Init other sensors
		robot.encoderController.reset();
		robot.gyro.reset();
		
		// Start guidance
		robot.guidanceData.start();
	}
	
	std::string name() const override {return "StartState";}
};
