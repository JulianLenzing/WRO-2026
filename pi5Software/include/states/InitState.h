#pragma once

#include "State.h"
#include "gpioControl.h"

class InitState : public State{
	void enter(RobotSystem& robot) override
    {
        robot.lidarDriver = *sl::createLidarDriver();
        if(!robot.lidarDriver) printf("Lidar driver not initialized\n");
        robot.displayUI.update();
        //robot.gp.setPosition(800, 0); Only on X11 
        dpd.clear();
        robot.gp.update(dpd);

		robot.visibility.setLineVisibility(SLAM_DEBUG_LINE, true);
        robot.startTime = std::chrono::high_resolution_clock::now();
    }

    void update(RobotSystem& robot) override
    {
        // transition condition handled externally
    }
    
    std::string name() const override {return "InitState";}
};
