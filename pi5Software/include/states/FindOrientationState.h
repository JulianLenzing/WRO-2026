#pragma once

#include <chrono>

#include "State.h"
#include "RobotSystem.h"
#include "LidarPoint.h"

class FindOrientationState : public State{
    void enter(RobotSystem& robot) override
    {
        // Determine run direction
        robot.initSlam.minPointDistance = 0.05f;
		robot.initSlam.maxDistanceDeviation = 0.7f;
    }

    bool update(RobotSystem& robot) override
    {
        LidarScan lidarScan;
        robot.lidar.getScan(lidarScan);
        printf("Determining run direction\n");
        if(!robot.initSlam.getRunDirection(robot.position, robot.heading, lidarScan, robot.runType, robot.runDirection, robot.doUnparking)) return false;

        printf("Run direction: ");
        if(robot.runDirection == RUN_DIRECTION_CCW) printf("CCW\n");
        else printf("CW\n");
 
        if (robot.runDirection == RUN_DIRECTION_CCW) robot.heading = 0;
        else robot.heading = M_PI;
        robot.pathfinder.setRunDirection(robot.runDirection);
        
        robot.initSlam.minPointDistance = robot.slam.minPointDistance;
		robot.initSlam.maxDistanceDeviation = 0.7f;
        return true;
    }

    void exit(RobotSystem& robot) override
    {
        robot.position = Vec2f(1.5f, 0.5f);
    }

    std::string name() const override {return "Find orientation state";}
};
