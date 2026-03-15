#pragma once

#include "State.h"

class StartState : public State{
    void update(RobotSystem& robot) override
    {
		if(robot.gpioController.queryButton()) robot.startActivated = true;
    }
    
    void exit(RobotSystem& robot) override 
    {
		// Start Lidar
		startLidar(robot.lidarDriver);
		
		// Start guidance
		//robot.guidanceData.start();
	}
	
	std::string name() const override {return "StartState";}
};
