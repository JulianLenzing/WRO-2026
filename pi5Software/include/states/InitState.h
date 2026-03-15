#pragma once

#include "State.h"
#include "gpioControl.h"

class InitState : public State{
	void enter(RobotSystem& robot) override
    {
        robot.lidarDriver = *sl::createLidarDriver();
        if(!robot.lidarDriver) printf("Lidar driver not initialized\n");

        initPoseEstimation();

		robot.visibility.setLineVisibility(SLAM_DEBUG_LINE,false);
    }

    void update(RobotSystem& robot) override
    {
        // transition condition handled externally
    }
    
    std::string name() const override {return "InitState";}
};
