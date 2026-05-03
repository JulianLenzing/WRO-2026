#include "sensorUpdateFunctions.h"

#include "../include/RobotSystem.h"

#define LIDAR_POSITION_TAU 0.001f
#define LIDAR_HEADING_TAU 0.8f

// Helper
Vec2f boundPosition(Vec2f position, Environment environment) {
    position.x = std::max(environment.outerBottomLeft.x, std::min(position.x, environment.outerTopRight.x));
    position.y = std::max(environment.outerBottomLeft.y, std::min(position.y, environment.outerTopRight.y));
    return position;
}

void updateGyro(RobotSystem& robot)
{
    float deltaHeading = 0.0f;
    if(robot.gyro.getDeltaHeading(deltaHeading)) {
        robot.heading += deltaHeading;
        robot.displayUI.gyroStatus = true;
    }
    else robot.displayUI.gyroStatus = false;
    robot.heading = Gyro::normaliseAngle(robot.heading);
}

void updateEncoder(RobotSystem& robot)
{
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

void updateLidar(RobotSystem& robot, std::chrono::milliseconds lidarDt)
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
    robot.slam.getUsablePoints(lidarScan, robot.position, robot.environment, useableScan);

    std::optional<float> lidarHeading;
    lidarHeading.reset();
    std::optional<float> maybeNewEstimatedHeading = robot.slam.lidarEstimateHeading(useableScan, robot.environment, robot.position);
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
        robot.slam.getUsablePoints(lidarScan, robot.position, robot.environment, useableScan);

        robot.displayUI.lidarHeadingStatus = true;
    }
    else {
        robot.displayUI.lidarHeadingStatus = false;
    }
    for(const auto& lp : lidarScan.scan) {dpd.appendPoint(lp.point() + robot.position, GRAY, UNUSEABLE_LIDAR_POINT_POINT);}
    for(const auto& lp : useableScan.scan) {dpd.appendPoint(lp.point() + robot.position, BLUE, USEABLE_LIDAR_POINT_POINT);}

    auto maybeNewEstimatedPosition = robot.slam.lidarEstimatePosition(useableScan, robot.environment, robot.position);

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
        robot.slam.getDistanceUseablePoints(lidarScan, useableScan);
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

void updateCamera(RobotSystem& robot)
{
    std::vector<Obstacle> obstacles;
    std::vector<Obstacle> filteredObstacles;
    robot.obstacleDetection.getObstacles(obstacles);
    robot.pathfinder.filterObstacles(obstacles, filteredObstacles);
    robot.obstacleDetection.feedImage(robot.camera.grabFrame(), filteredObstacles, robot.position, robot.heading);
}
