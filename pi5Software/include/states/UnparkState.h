#pragma once

#include <chrono>

#include "State.h"
#include "RobotSystem.h"
#include "LidarPoint.h"
#include "sensorUpdateFunctions.h"
#include "Timer.h"

// Update times in ms
constexpr int UNPARKING_GYRO_UPDATE_TIME = 10;
constexpr int UNPARKING_ENCODER_UPDATE_TIME = 10;

class UnparkState : public State{
public:
	UnparkState()
		:
		gyroTimer(UNPARKING_GYRO_UPDATE_TIME),
        encoderTimer(UNPARKING_ENCODER_UPDATE_TIME)
	{}
	
    void enter(RobotSystem& robot) override
    {
		robot.position = Vec2f(2.0f - robot.length / 2.0f, 0.1f);
		robot.guidanceData.setRobotData(robot.position, robot.heading);
		robot.pathfinder.appendUnparkingPath(robot.guidanceData);
		
		gyroTimer.reset();
        encoderTimer.reset();
    }

    bool update(RobotSystem& robot) override
    {       
		#ifndef USE_ENCODER_FOR_HEADING
        /*----------Gyro-loop----------*/
        if (gyroTimer.isExpired()) {
            updateGyro(robot);
            gyroTimer.reset();
        }
        #endif

        /*----------Encoder-loop---------*/
        if (encoderTimer.isExpired()) {
            updateEncoder(robot);
            encoderTimer.reset();
        }
		robot.guidanceData.setRobotData(robot.position, robot.heading);
		if(robot.guidanceData.getReachedLastWaypoint()) return 1;
		return 0;
    }

    void exit(RobotSystem& robot) override
    {
    }

    std::string name() const override {return "Unpark state";}
    
private:
	TimerMillis gyroTimer;
	TimerMillis encoderTimer;
};
