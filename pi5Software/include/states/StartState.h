#pragma once

#include <chrono>

#include "State.h"
#include "RobotSystem.h"

class StartState : public State{
    void update(RobotSystem& robot) override
    {
		if(robot.gpioController.queryButton()) robot.startActivated = true;
    }
    
    void exit(RobotSystem& robot) override 
    {
		robot.startTime = std::chrono::high_resolution_clock::now();

		// Start Lidar
		startLidar(robot.lidarDriver);
		
		// Start guidance
		robot.guidanceData.start();
	}
	
	std::string name() const override {return "StartState";}
};
