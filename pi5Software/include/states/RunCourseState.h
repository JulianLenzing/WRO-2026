#pragma once

#include <cmath>

#include "State.h"
#include "RobotSystem.h"
#include "slam.h"
#include "sensorUpdateFunctions.h"
#include "Timer.h"

// Update times in ms
constexpr int GYRO_UPDATE_TIME = 10;
constexpr int ENCODER_UPDATE_TIME = 10;
constexpr int LIDAR_UPDATE_TIME = 105;
constexpr int CAMERA_UPDATE_TIME = 500;
constexpr int GUIDANCE_UPDATE_TIME = 50;
constexpr int UI_UPDATE_TIME = 500;

class RunCourseState : public State{
public:
    RunCourseState()
        :
        gyroTimer(GYRO_UPDATE_TIME),
        encoderTimer(ENCODER_UPDATE_TIME),
        lidarTimer(LIDAR_UPDATE_TIME),
        cameraTimer(CAMERA_UPDATE_TIME),
        guidanceTimer(GUIDANCE_UPDATE_TIME),
        uITimer(UI_UPDATE_TIME)
    {}

    void enter(RobotSystem& robot) override
    {
        gyroTimer.reset();
        encoderTimer.reset();
        lidarTimer.reset();
        cameraTimer.reset();
        guidanceTimer.reset();
        uITimer.reset();
    }

    void update(RobotSystem& robot) override
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

        /*----------Lidar-loop---------*/
        if (lidarTimer.isExpired()) {
            updateLidar(robot, lidarTimer.passedMs());
            lidarTimer.reset();
        }

        /*----------Camera-loop---------*/
        if (robot.runType == RUN_TYPE_OBSTACLE_RUN)
        {
            if(cameraTimer.isExpired()) {
                updateCamera(robot);
                cameraTimer.reset();
            }
        }

        /*----------Guidance-loop---------*/
        if(guidanceTimer.isExpired()) {
            // Update guidance data with the new position and heading            
            robot.guidanceData.setRobotData(robot.position, robot.heading);

            // Update waypoints using obstacles
            std::vector<Obstacle> obstacles;
            robot.obstacleDetection.getObstacles(obstacles);
            if (robot.guidanceData.getReachedLastWaypoint())
            {
                robot.pathfinder.update(robot.position, robot.heading, obstacles, robot.guidanceData);
            }

            if (robot.pathfinder.shouldStop()) robot.stopRequested = true;

            guidanceTimer.reset();
        }

        /*----------UI-loop---------*/
        if (uITimer.isExpired()) {
            float steeringAngle, throttle;
            robot.guidanceData.getUiData(steeringAngle, throttle);

            robot.displayUI.position = robot.position;
            robot.displayUI.heading = robot.heading;
            robot.displayUI.steeringAngle = steeringAngle;
            robot.displayUI.throttle = throttle;
            robot.displayUI.currentWaypoint = robot.guidanceData.lookAtCurrentWaypoint().point;
            robot.displayUI.round = robot.pathfinder.getRound();
            robot.displayUI.update();

            uITimer.reset();
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    
    std::string name() const override {return "RunCourseState";}

    private:
	TimerMillis gyroTimer;
    TimerMillis encoderTimer;
    TimerMillis lidarTimer;
    TimerMillis cameraTimer;
    TimerMillis guidanceTimer;
    TimerMillis uITimer;
};
