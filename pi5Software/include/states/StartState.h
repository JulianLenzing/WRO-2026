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
		robot.pathfinder.setRunDirection(RUN_DIRECTION_CCW); // Placeholder until run direction is automatically determined

		// Start Lidar
		startLidar(robot.lidarDriver);

		// Init other sensors
		robot.encoderController.reset();
		robot.gyro.reset();
		
		// Start guidance
		robot.guidanceData.setRobotData(robot.position, robot.heading);
		robot.guidanceData.start();
	}
	
	std::string name() const override {return "StartState";}
};
