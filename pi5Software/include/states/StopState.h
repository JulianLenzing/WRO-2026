#pragma once

#include "State.h"

class StopState : public State{
	void enter(RobotSystem& robot) override
  {
    // Stop sensors
    stopLidar(robot.lidarDriver);

    // Stop other threads
    robot.guidanceData.terminate();
    if (robot.guidanceThread.joinable()) {
    	robot.guidanceThread.join();
    }
  }

  void update(RobotSystem& robot) override {
    // transition condition handled externally
  }
    
  std::string name() const override {return "StopState";}
};
