#pragma once
#pragma once

#include <chrono>

#include "State.h"
#include "RobotSystem.h"
#include "slam.h"
#include "sensorUpdateFunctions.h"
#include "LidarPoint.h"
#include "../RobotSystem.h"

class FindPositionState : public State{
    void enter(RobotSystem& robot) override
    {
        robot.position = Vec2f(1.5f, 0.5f);

        // Determine run direction
        LidarScan lidarScan;
        do
        {
            getLidarScan(robot.lidarDriver, lidarScan, 1, 0.25);
            printf("Determining run direction\n");
        }
        while (!getRunDirection(robot.position, robot.heading, lidarScan, robot.runDirection));
        if (robot.runDirection == RUN_DIRECTION_CCW) robot.heading = 0;
        else robot.heading = M_PI;
        robot.pathfinder.setRunDirection(robot.runDirection);
    }

    bool update(RobotSystem& robot) override
    {
        if (iterations < 10)
        {
            // Setup graphics for new frame
            dpd.clear();
            dpd.appendPoint(robot.position, RED, ESTIMATED_POSITION_POINT);
            float length = 0.15f;
            dpd.appendLine(Line(robot.position, Vec2f(robot.position.x + cos(robot.heading) * length, robot.position.y + sin(robot.heading) * length)), RED);
            for (int i = 0; i < robot.environment.landmarks.size(); i++)
            {
                dpd.appendLine(robot.environment.landmarks[i].line, WHITE, LANDMARK_LINE);
            }

            LidarScan lidarScan;
            getLidarScan(robot.lidarDriver, lidarScan, 1, 0.25);
            lidarScan.rotate(robot.heading); // Rotate scan to align with robot's heading
            float beginningHeading = robot.heading;

            LidarScan useableScan;
            getUsablePoints(lidarScan, robot.position, robot.environment, useableScan, 0.2f, 0.7f);

            std::optional<float> lidarHeading;
            lidarHeading.reset();
            std::optional<float> maybeNewEstimatedHeading = lidarEstimateHeading(useableScan, robot.environment, robot.position);
             if(maybeNewEstimatedHeading.has_value()) {
                float error = maybeNewEstimatedHeading.value();
                lidarHeading = robot.heading + error;
                robot.heading += error;
                robot.heading = EncoderController::normaliseAngle(robot.heading);

                // The scan is corrected using the angle error from the lidar
                // The useable points are reassigned to ensure greater accuracy
                lidarScan.rotate(error);
                useableScan.scan.clear();
                getUsablePoints(lidarScan, robot.position, robot.environment, useableScan, 0.2f, 0.7f);
             }
            for(const auto& lp : lidarScan.scan) {dpd.appendPoint(lp.point() + robot.position, GRAY, UNUSEABLE_LIDAR_POINT_POINT);}
            for(const auto& lp : useableScan.scan) {dpd.appendPoint(lp.point() + robot.position, BLUE, USEABLE_LIDAR_POINT_POINT);}

            auto maybeNewEstimatedPosition = lidarEstimatePosition(useableScan, robot.environment, robot.position);

            if(maybeNewEstimatedPosition.has_value()) {
                robot.position = maybeNewEstimatedPosition.value();

                Vec2f tmp = maybeNewEstimatedPosition.value();
                dpd.appendPoint(tmp, YELLOW, NEW_ESTIMATED_POSITION_POINT);
                if(lidarHeading.has_value()) dpd.appendLine(Line(tmp, Vec2f(tmp.x + cos(lidarHeading.value()) * length, tmp.y + sin(lidarHeading.value()) * length)), YELLOW);
            }

            robot.gp.update(dpd);
            iterations++;
            std::this_thread::sleep_for(std::chrono::milliseconds(120));
        }
        else return true;
        return false;
    }

    void exit(RobotSystem& robot) override
    {
        robot.pathfinder.setStartingPosition(robot.position);
    }

    std::string name() const override {return "Find position state";}

private:
    int iterations = 0;
};
