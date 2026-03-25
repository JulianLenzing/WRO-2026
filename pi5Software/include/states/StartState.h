#pragma once

#include <chrono>

#include "State.h"
#include "RobotSystem.h"

class StartState : public State{
    void update(RobotSystem& robot) override
    {
		if(robot.gpioController.queryButton()) robot.startActivated = true;
		robot.displayUI.update();
		std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    
    void exit(RobotSystem& robot) override 
    {
		robot.startTime = std::chrono::high_resolution_clock::now();

		// Start Lidar
		startLidar(robot.lidarDriver);

		// Init Encoder angle
		robot.encoderController.reset();
		
		// Start guidance
		robot.guidanceData.start();
	}
	
	std::string name() const override {return "StartState";}
};
