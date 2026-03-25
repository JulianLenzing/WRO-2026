#pragma once

#include <cmath>

#include "State.h"
#include "RobotSystem.h"
#include "slam.h"

#define LIDAR_POSITION_TAU 1.0f
#define LIDAR_HEADING_TAU 1.0f

void writeLidarScanToFile(const LidarScan& scan, const std::string& filename)
{
    static std::ofstream out(filename, std::ios::app); // append mode

    if (!out.is_open()) {
        std::cerr << "Failed to open file: " << filename << std::endl;
        return;
    }

    for (const auto& p : scan.scan) {
        out << std::fixed << std::setprecision(2)
            << p.distance << " "
            << p.angle << "\n";
    }

    out << "----\n"; // separator between scans (optional)
}

class RunCourseState : public State{
    void enter(RobotSystem& robot) override
    {
        lastEncoderUpdateTime = std::chrono::high_resolution_clock::now();
        lastLidarUpdateTime = std::chrono::high_resolution_clock::now();   
        lastGuidanceUpdateTime = std::chrono::high_resolution_clock::now();
        lastUIUpdateTime = std::chrono::high_resolution_clock::now();   

        robot.position = Vec2f(0.5f, 0.5f);
        robot.heading = 0.0f;

        robot.guidanceData.appendWaypoint(Vec2f(2.0f, 0.5f));
        robot.guidanceData.appendWaypoint(Vec2f(2.5f, 1.0f));
        robot.guidanceData.appendWaypoint(Vec2f(2.5f, 2.0f));
        robot.guidanceData.appendWaypoint(Vec2f(2.0f, 2.5f));
        robot.guidanceData.appendWaypoint(Vec2f(1.0f, 2.5f));
        robot.guidanceData.appendWaypoint(Vec2f(0.5f, 2.0f));
        robot.guidanceData.appendWaypoint(Vec2f(0.5f, 1.0f));
        robot.guidanceData.appendWaypoint(Vec2f(1.0f, 0.5f));
    }

    void update(RobotSystem& robot) override
    {
        std::chrono::high_resolution_clock::time_point now =
        std::chrono::high_resolution_clock::now();

        /*----------Encoder-loop---------*/
        std::chrono::milliseconds EncoderDt = std::chrono::duration_cast<std::chrono::milliseconds>(now - lastEncoderUpdateTime);
        if (EncoderDt >= std::chrono::milliseconds(10)) {
            lastEncoderUpdateTime = now;
            
            // Update position and heading based on encoder data
            float deltaDistance;
            float deltaHeading;
            robot.encoderController.getEncodingData(deltaDistance, deltaHeading);
            float midHeading = robot.heading + deltaHeading * 0.5f;
            robot.position += Vec2f(cosf(midHeading), sinf(midHeading)) * deltaDistance;
            robot.position = boundPosition(robot.position, robot.landmarks);
            robot.heading += deltaHeading;
            robot.heading =  EncoderController::normaliseAngle(robot.heading);

            printf("---------Encoder--------------\n");
            printf("T1: %d ms T2: %d ms DT: %d ms\n", std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - robot.initTime).count(), std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - robot.startTime).count(), EncoderDt.count());

            cout << "Position: " << robot.position.x << ", " << robot.position.y << " m, Heading: " << robot.heading / M_PI * 180 << " degrees" << endl;
        }

        /*----------Lidar-loop---------*/
        std::chrono::milliseconds lidarDt = std::chrono::duration_cast<std::chrono::milliseconds>(now - lastLidarUpdateTime);
        if (lidarDt >= std::chrono::milliseconds(105)) {
            lastLidarUpdateTime = now;

            printf("-----------Lidar---------------\n");
            printf("T1: %d ms T2: %d ms DT: %d ms\n", std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - robot.initTime).count(), std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - robot.startTime).count(), lidarDt.count());

            dpd.clear();
            dpd.appendPoint(robot.position, RED, ESTIMATED_POSITION_POINT);
            dpd.appendLines(robot.landmarks.lines, WHITE, LANDMARK_LINE);
            
            LidarScan lidarScan;
            getLidarScan(robot.lidarDriver, lidarScan, 1, 0.25);
            lidarScan.rotate(robot.heading); // Rotate scan to align with robot's heading
            float beginningHeading = robot.heading;

            LidarScan useableScan;
            getUsablePoints(lidarScan, robot.position, robot.landmarks, useableScan);

            std::optional<float> maybeNewEstimatedHeading = lidarEstimateHeading(useableScan, robot.landmarks, robot.position);            
             if(maybeNewEstimatedHeading.has_value()) {
                float error = maybeNewEstimatedHeading.value();
                float alpha = std::exp(-lidarDt.count() / 1000.0f / LIDAR_HEADING_TAU);
                robot.heading += error * (1.0f - alpha);
                robot.heading = EncoderController::normaliseAngle(robot.heading);
                useableScan.rotate(robot.heading-beginningHeading);
                cout << "Lidar heading: " << maybeNewEstimatedHeading.value() / M_PI * 180 << " degrees" << endl;
            }
            else {
                cout << "Heading estimation failed, keeping previous estimate." << endl;
            }
            for(const auto& lp : lidarScan.scan) {dpd.appendPoint(lp.point() + robot.position, GRAY, UNUSEABLE_LIDAR_POINT_POINT);}
            for(const auto& lp : useableScan.scan) {dpd.appendPoint(lp.point() + robot.position, BLUE, USEABLE_LIDAR_POINT_POINT);}


            auto maybeNewEstimatedPosition = lidarEstimatePosition(useableScan, robot.landmarks, robot.position);

            if(maybeNewEstimatedPosition.has_value()) {
                Vec2f error = maybeNewEstimatedPosition.value() - robot.position;
                dpd.appendPoint(maybeNewEstimatedPosition.value(), YELLOW, NEW_ESTIMATED_POSITION_POINT);

                float alpha = std::exp(-lidarDt.count() / 1000.0f / LIDAR_POSITION_TAU);
                robot.position += error * (1.0f - alpha);
                robot.position = boundPosition(robot.position, robot.landmarks);

                cout << "Lidar position: " << maybeNewEstimatedPosition.value().x << ", " << maybeNewEstimatedPosition.value().y << " m" << endl;
            }
            else {
                cout << "Position estimation failed, keeping previous estimate." << endl;
            }

            robot.gp.update(dpd);
        }

        /*----------Guidance-loop---------*/
        std::chrono::milliseconds guidanceDt = std::chrono::duration_cast<std::chrono::milliseconds>(now - lastUIUpdateTime);
        if(guidanceDt >= std::chrono::milliseconds(50)) {
            lastGuidanceUpdateTime = now;

            // Update guidance data with the new position and heading            
            robot.guidanceData.setRobotData(robot.position, robot.heading);
            if(0) robot.guidanceData.appendWaypoint(Vec2f(2.5, 0.5)); // Placeholder for when we have a way to determine new waypoints
        }

        /*----------UI-loop---------*/
        std::chrono::milliseconds uIDt = std::chrono::duration_cast<std::chrono::milliseconds>(now - lastUIUpdateTime);
        if (uIDt >= std::chrono::milliseconds(500)) {
            lastUIUpdateTime = now;

            float steeringAngle, throttle;
            robot.guidanceData.getUiData(steeringAngle, throttle);

            robot.displayUI.position = robot.position;
            robot.displayUI.heading = robot.heading;
            robot.displayUI.steeringAngle = steeringAngle;
            robot.displayUI.throttle = throttle;
            robot.displayUI.update();
            dpd.updateVisibility(robot.visibility);	
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    
    std::string name() const override {return "RunCourseState";}

    private:
	std::chrono::high_resolution_clock::time_point lastLidarUpdateTime;
    std::chrono::high_resolution_clock::time_point lastEncoderUpdateTime;
    std::chrono::high_resolution_clock::time_point lastGuidanceUpdateTime;
    std::chrono::high_resolution_clock::time_point lastUIUpdateTime;

    Vec2f boundPosition(Vec2f position, Landmarks lms) {
        position.x = std::max(lms.outerBottomLeft.x, std::min(position.x, lms.outerTopRight.x));
        position.y = std::max(lms.outerBottomLeft.y, std::min(position.y, lms.outerTopRight.y));
        return position;
    }
};
