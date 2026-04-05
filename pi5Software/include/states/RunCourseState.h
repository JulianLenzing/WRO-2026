#pragma once

#include <cmath>

#include "State.h"
#include "RobotSystem.h"
#include "slam.h"

//#define USE_ENCODER_FOR_HEADING

#define LIDAR_POSITION_TAU 0.4f
#define LIDAR_HEADING_TAU 0.1f

// Update times in ms
#define ENCODER_UPDATE_TIME 10
#define GYRO_UPDATE_TIME 10
#define LIDAR_UPDATE_TIME 105
#define GUIDANCE_UPDATE_TIME 50
#define UI_UPDATE_TIME 500

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

        robot.position = Vec2f(1.0f, 0.25f);
        robot.heading = 0.0f;

        for(int i = 0; i < 3; i++) {
            /*
            robot.guidanceData.appendWaypoint(Vec2f(2.0f, 0.5f));
            robot.guidanceData.appendWaypoint(Vec2f(2.5f, 1.0f));
            robot.guidanceData.appendWaypoint(Vec2f(2.5f, 2.0f));
            robot.guidanceData.appendWaypoint(Vec2f(2.0f, 2.5f));
            robot.guidanceData.appendWaypoint(Vec2f(1.0f, 2.5f));
            robot.guidanceData.appendWaypoint(Vec2f(0.5f, 2.0f));
            robot.guidanceData.appendWaypoint(Vec2f(0.5f, 1.0f));
            robot.guidanceData.appendWaypoint(Vec2f(1.0f, 0.5f));
            */
            robot.guidanceData.appendWaypoint(Vec2f(1.5f, 0.25f));
            robot.guidanceData.appendWaypoint(Vec2f(1.75f, 0.5f));
            robot.guidanceData.appendWaypoint(Vec2f(1.75f, 1.5f));
            robot.guidanceData.appendWaypoint(Vec2f(1.5f, 1.75f));
            robot.guidanceData.appendWaypoint(Vec2f(0.5f, 1.75f));
            robot.guidanceData.appendWaypoint(Vec2f(0.25f, 1.5f));
            robot.guidanceData.appendWaypoint(Vec2f(0.25f, 0.5f));
            robot.guidanceData.appendWaypoint(Vec2f(0.5f, 0.25f));
            
        }
        robot.guidanceData.appendWaypoint(Vec2f(1.0f, 0.25f));
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

            printf("-----------Gyro---------------\n");
            printf("T1: %d ms T2: %d ms DT: %d ms\n", std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - robot.initTime).count(), std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - robot.startTime).count(), gyroDt.count());
            float deltaHeading = 0.0f;
            if(robot.gyro.getDeltaHeading(deltaHeading)) {
                robot.heading += deltaHeading;
                robot.displayUI.gyroStatus = true;
            }
            else robot.displayUI.gyroStatus = false;
            robot.heading = Gyro::normaliseAngle(robot.heading);
            printf("Delta Yaw: %.2f Absolute Yaw: %.2f\n", deltaHeading, robot.heading);
        }
        #endif

        /*----------Encoder-loop---------*/
        std::chrono::milliseconds EncoderDt = std::chrono::duration_cast<std::chrono::milliseconds>(now - lastEncoderUpdateTime);
        if (EncoderDt >= std::chrono::milliseconds(ENCODER_UPDATE_TIME)) {
            lastEncoderUpdateTime = now;
            
            printf("---------Encoder--------------\n");
            printf("T1: %d ms T2: %d ms DT: %d ms\n", std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - robot.initTime).count(), std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - robot.startTime).count(), EncoderDt.count());

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
                robot.position = boundPosition(robot.position, robot.landmarks);
            #endif
            #ifdef USE_ENCODER_FOR_HEADING
                float midHeading = robot.heading + deltaHeading * 0.5f;
                robot.position += Vec2f(cosf(midHeading), sinf(midHeading)) * deltaDistance;
                robot.position = boundPosition(robot.position, robot.landmarks);
                robot.heading += deltaHeading;
                robot.heading =  EncoderController::normaliseAngle(robot.heading);
            #endif
            cout << "Delta distance: " << deltaDistance << ", Position: " << robot.position.x << ", " << robot.position.y << " m, Heading: " << robot.heading / M_PI * 180 << " degrees" << endl;
        }

        /*----------Lidar-loop---------*/
        std::chrono::milliseconds lidarDt = std::chrono::duration_cast<std::chrono::milliseconds>(now - lastLidarUpdateTime);
        if (lidarDt >= std::chrono::milliseconds(LIDAR_UPDATE_TIME)) {
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
                printf("Error: %.2f Alpha: %.2f Added Heading: %.2f\n", error, alpha, error * (1.0f-alpha));
                useableScan.rotate(robot.heading-beginningHeading);
                cout << "Lidar heading: " << maybeNewEstimatedHeading.value() / M_PI * 180 << " degrees" << endl;
                robot.displayUI.lidarHeadingStatus = true;
            }
            else {
                cout << "Heading estimation failed, keeping previous estimate." << endl;
                robot.displayUI.lidarHeadingStatus = false;
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
                robot.displayUI.lidarPositionStatus = true;
            }
            else {
                cout << "Position estimation failed, keeping previous estimate." << endl;
                robot.displayUI.lidarPositionStatus = false;
            }

            dpd.updateVisibility(robot.visibility);	
            dpd.appendPoint(robot.guidanceData.lookAtCurrentWaypoint().point, MAGENTA, STANDARD_POINT);
            robot.gp.update(dpd);
        }

        /*----------Guidance-loop---------*/
        std::chrono::milliseconds guidanceDt = std::chrono::duration_cast<std::chrono::milliseconds>(now - lastUIUpdateTime);
        if(guidanceDt >= std::chrono::milliseconds(GUIDANCE_UPDATE_TIME)) {
            lastGuidanceUpdateTime = now;

            // Update guidance data with the new position and heading            
            robot.guidanceData.setRobotData(robot.position, robot.heading);
            if(0) robot.guidanceData.appendWaypoint(Vec2f(2.5, 0.5)); // Placeholder for when we have a way to determine new waypoints
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

    Vec2f boundPosition(Vec2f position, Landmarks lms) {
        position.x = std::max(lms.outerBottomLeft.x, std::min(position.x, lms.outerTopRight.x));
        position.y = std::max(lms.outerBottomLeft.y, std::min(position.y, lms.outerTopRight.y));
        return position;
    }
};
