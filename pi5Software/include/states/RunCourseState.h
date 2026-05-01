#pragma once

#include <cmath>

#include "State.h"
#include "RobotSystem.h"
#include "slam.h"
#include "../RobotSystem.h"

//#define USE_ENCODER_FOR_HEADING

#define LIDAR_POSITION_TAU 0.001f
#define LIDAR_HEADING_TAU 0.8f

// Update times in ms
#define ENCODER_UPDATE_TIME 10
#define GYRO_UPDATE_TIME 10
#define LIDAR_UPDATE_TIME 105
#define GUIDANCE_UPDATE_TIME 50
#define UI_UPDATE_TIME 500
#define CAMERA_UPDATE_TIME 500

class RunCourseState : public State{
    void enter(RobotSystem& robot) override
    {
        lastEncoderUpdateTime = std::chrono::high_resolution_clock::now();
        lastLidarUpdateTime = std::chrono::high_resolution_clock::now();   
        lastGuidanceUpdateTime = std::chrono::high_resolution_clock::now();
        lastUIUpdateTime = std::chrono::high_resolution_clock::now();
        lastCameraUpdateTime = std::chrono::high_resolution_clock::now();
    }

    void update(RobotSystem& robot) override
    {
        std::chrono::high_resolution_clock::time_point now =
        std::chrono::high_resolution_clock::now();

        #ifndef USE_ENCODER_FOR_HEADING
        /*----------Gyro-loop----------*/
        std::chrono::milliseconds gyroDt = std::chrono::duration_cast<std::chrono::milliseconds>(now - lastGyroUpdateTime);
        if (gyroDt >= std::chrono::milliseconds(GYRO_UPDATE_TIME)) {
            lastGyroUpdateTime = now;

            float deltaHeading = 0.0f;
            if(robot.gyro.getDeltaHeading(deltaHeading)) {
                robot.heading += deltaHeading;
                robot.displayUI.gyroStatus = true;
            }
            else robot.displayUI.gyroStatus = false;
            robot.heading = Gyro::normaliseAngle(robot.heading);
        }
        #endif

        /*----------Encoder-loop---------*/
        std::chrono::milliseconds encoderDt = std::chrono::duration_cast<std::chrono::milliseconds>(now - lastEncoderUpdateTime);
        if (encoderDt >= std::chrono::milliseconds(ENCODER_UPDATE_TIME)) {
            lastEncoderUpdateTime = now;

            // Update position and heading based on encoder data
            float deltaDistance = 0;
            float deltaHeading = 0;
            if(!robot.encoderController.getEncodingData(deltaDistance, deltaHeading)) {
                printf("Could not grab encoder data!\n");
                robot.displayUI.encoderStatus = false;
            }            
            else robot.displayUI.encoderStatus = true;
            #ifndef USE_ENCODER_FOR_HEADING
                robot.position += Vec2f(cosf(robot.heading), sinf(robot.heading)) * deltaDistance;
                robot.position = boundPosition(robot.position, robot.environment);
            #endif
            #ifdef USE_ENCODER_FOR_HEADING
                float midHeading = robot.heading + deltaHeading * 0.5f;
                robot.position += Vec2f(cosf(midHeading), sinf(midHeading)) * deltaDistance;
                robot.position = boundPosition(robot.position, robot.landmarks);
                robot.heading += deltaHeading;
                robot.heading =  EncoderController::normaliseAngle(robot.heading);
            #endif
        }

        /*----------Lidar-loop---------*/
        std::chrono::milliseconds lidarDt = std::chrono::duration_cast<std::chrono::milliseconds>(now - lastLidarUpdateTime);
        if (lidarDt >= std::chrono::milliseconds(LIDAR_UPDATE_TIME)) {
            lastLidarUpdateTime = now;

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
            getUsablePoints(lidarScan, robot.position, robot.environment, useableScan);
            
            std::optional<float> lidarHeading;
            lidarHeading.reset();
            std::optional<float> maybeNewEstimatedHeading = lidarEstimateHeading(useableScan, robot.environment, robot.position);
             if(maybeNewEstimatedHeading.has_value()) {
                float error = maybeNewEstimatedHeading.value();
                lidarHeading = robot.heading + error;
                float alpha = std::exp(-lidarDt.count() / 1000.0f / LIDAR_HEADING_TAU);
                robot.heading += error * (1.0f - alpha);
                robot.heading = EncoderController::normaliseAngle(robot.heading);

                // The scan is corrected using the angle error from the lidar
                // The useable points are reassigned to ensure greater accuracy             
                lidarScan.rotate(error);
                useableScan.scan.clear();
                getUsablePoints(lidarScan, robot.position, robot.environment, useableScan);

                robot.displayUI.lidarHeadingStatus = true;
            }
            else {
                robot.displayUI.lidarHeadingStatus = false;
            }
            for(const auto& lp : lidarScan.scan) {dpd.appendPoint(lp.point() + robot.position, GRAY, UNUSEABLE_LIDAR_POINT_POINT);}
            for(const auto& lp : useableScan.scan) {dpd.appendPoint(lp.point() + robot.position, BLUE, USEABLE_LIDAR_POINT_POINT);}

            auto maybeNewEstimatedPosition = lidarEstimatePosition(useableScan, robot.environment, robot.position);

            if(maybeNewEstimatedPosition.has_value()) {
                Vec2f error = maybeNewEstimatedPosition.value() - robot.position;
                float alpha = std::exp(-lidarDt.count() / 1000.0f / LIDAR_POSITION_TAU);
                robot.position += error * (1.0f - alpha);
                robot.position = boundPosition(robot.position, robot.environment);

                Vec2f tmp = maybeNewEstimatedPosition.value();
                dpd.appendPoint(tmp, YELLOW, NEW_ESTIMATED_POSITION_POINT);
                if(lidarHeading.has_value()) dpd.appendLine(Line(tmp, Vec2f(tmp.x + cos(lidarHeading.value()) * length, tmp.y + sin(lidarHeading.value()) * length)), YELLOW);

                robot.displayUI.lidarPositionStatus = true;
            }
            else {
                robot.displayUI.lidarPositionStatus = false;
            }

            /*---------Detect-obstacles----------*/
            if (robot.runType == RUN_TYPE_OBSTACLE_RUN)
            {
                useableScan.scan.clear();
                getDistanceUseablePoints(lidarScan, useableScan);
                robot.obstacleDetection.feedScan(useableScan, robot.position);
                for(const Obstacle& o : robot.obstacleDetection.possibleObstacles) {
                    dpd.appendPoint(o.position, GRAY);
                }
                std::vector<Obstacle> obstacles;
                robot.obstacleDetection.getObstacles(obstacles);
                std::vector<Obstacle> filteredObstacles;
                robot.pathfinder.filterObstacles(obstacles, filteredObstacles);
                for(const Obstacle& o : filteredObstacles) {
                    if (o.getColor() == OBSTACLE_COLOUR_RED) dpd.appendPoint(o.position, RED);
                    else if (o.getColor() == OBSTACLE_COLOUR_GREEN) dpd.appendPoint(o.position, GREEN);
                    else dpd.appendPoint(o.position, YELLOW);
                }
            }

            /*--------Update-graphics-----------*/
            robot.visibility.setLineVisibility(SLAM_DEBUG_LINE, false);
            dpd.updateVisibility(robot.visibility);	
            dpd.appendPoint(robot.guidanceData.lookAtCurrentWaypoint().point, MAGENTA);
            robot.gp.update(dpd);
        }

        /*----------Camera-loop---------*/
        if (robot.runType == RUN_TYPE_OBSTACLE_RUN)
        {
            std::chrono::milliseconds cameraDt = std::chrono::duration_cast<std::chrono::milliseconds>(now - lastCameraUpdateTime);
            if(cameraDt >= std::chrono::milliseconds(CAMERA_UPDATE_TIME)) {
                lastCameraUpdateTime = now;

                std::vector<Obstacle> obstacles;
                std::vector<Obstacle> filteredObstacles;
                robot.obstacleDetection.getObstacles(obstacles);
                robot.pathfinder.filterObstacles(obstacles, filteredObstacles);
                robot.obstacleDetection.feedImage(robot.camera.grabFrame(), filteredObstacles, robot.position, robot.heading);
            }
        }

        /*----------Guidance-loop---------*/
        std::chrono::milliseconds guidanceDt = std::chrono::duration_cast<std::chrono::milliseconds>(now - lastGuidanceUpdateTime);
        if(guidanceDt >= std::chrono::milliseconds(GUIDANCE_UPDATE_TIME)) {
            lastGuidanceUpdateTime = now;

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
        }

        /*----------UI-loop---------*/
        std::chrono::milliseconds uIDt = std::chrono::duration_cast<std::chrono::milliseconds>(now - lastUIUpdateTime);
        if (uIDt >= std::chrono::milliseconds(UI_UPDATE_TIME)) {
            lastUIUpdateTime = now;

            float steeringAngle, throttle;
            robot.guidanceData.getUiData(steeringAngle, throttle);

            robot.displayUI.position = robot.position;
            robot.displayUI.heading = robot.heading;
            robot.displayUI.steeringAngle = steeringAngle;
            robot.displayUI.throttle = throttle;
            robot.displayUI.currentWaypoint = robot.guidanceData.lookAtCurrentWaypoint().point;
            robot.displayUI.round = robot.pathfinder.getRound();

            robot.displayUI.update();
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    
    std::string name() const override {return "RunCourseState";}

    private:
	std::chrono::high_resolution_clock::time_point lastLidarUpdateTime;
    std::chrono::high_resolution_clock::time_point lastGyroUpdateTime;
    std::chrono::high_resolution_clock::time_point lastEncoderUpdateTime;
    std::chrono::high_resolution_clock::time_point lastGuidanceUpdateTime;
    std::chrono::high_resolution_clock::time_point lastUIUpdateTime;
    std::chrono::high_resolution_clock::time_point lastCameraUpdateTime;

    Vec2f boundPosition(Vec2f position, Environment environment) {
        position.x = std::max(environment.outerBottomLeft.x, std::min(position.x, environment.outerTopRight.x));
        position.y = std::max(environment.outerBottomLeft.y, std::min(position.y, environment.outerTopRight.y));
        return position;
    }
};
