#pragma once

#include "State.h"

class StopState : public State{
	void enter(RobotSystem& robot) override
  {
    // Stop sensors
    robot.lidar.stop();

    // Stop other threads
    robot.guidanceData.terminate();
    if (robot.guidanceThread.joinable()) {
    	robot.guidanceThread.join();
    }
  }

  bool update(RobotSystem& robot) override {
    return true;
  }
    
  std::string name() const override {return "StopState";}
};
