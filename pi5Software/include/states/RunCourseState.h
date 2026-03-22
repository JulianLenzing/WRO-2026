#pragma once

#include <cmath>

#include "State.h"
#include "RobotSystem.h"

#define LIDAR_TAU 1.0f

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
        lastLidarUpdateTime = std::chrono::high_resolution_clock::now();   
        lastEncoderUpdateTime = std::chrono::high_resolution_clock::now();   
        lastUIUpdateTime = std::chrono::high_resolution_clock::now();   

        robot.position = Vec2f(0.5, 0.5);
        robot.heading = 0.0f;
    }

    void update(RobotSystem& robot) override
    {
        std::chrono::high_resolution_clock::time_point now =
        std::chrono::high_resolution_clock::now();

        std::chrono::milliseconds EncoderDt = std::chrono::duration_cast<std::chrono::milliseconds>(now - lastEncoderUpdateTime);
        if (EncoderDt >= std::chrono::milliseconds(10)) {
            lastEncoderUpdateTime = now;
            
            float deltaDistance;
            robot.encoderController.getEncodingData(deltaDistance, robot.heading);
            robot.position += Vec2f(cosf(robot.heading), sinf(robot.heading)) * deltaDistance;

            printf("---------Encoder--------------\n");
            printf("T1: %d ms T2: %d ms\n", std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - robot.initTime).count(), std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - robot.startTime).count());

            cout << "Position: " << robot.position.x << ", " << robot.position.y << " m, Heading: " << robot.heading / M_PI * 180 << " degrees" << endl;
        }

        std::chrono::milliseconds lidarDt = std::chrono::duration_cast<std::chrono::milliseconds>(now - lastLidarUpdateTime);
        if (lidarDt >= std::chrono::milliseconds(105)) {
            lastLidarUpdateTime = now;

            printf("-----------Lidar---------------\n");
            printf("T1: %d ms T2: %d ms\n", std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - robot.initTime).count(), std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - robot.startTime).count());

            dpd.clear();
            dpd.appendPoint(robot.position, RED, ESTIMATED_POSITION_POINT);
            dpd.appendLines(robot.landmarks.lines, WHITE, LANDMARK_LINE);
            
            LidarScan lidarScan;
            getLidarScan(robot.lidarDriver, lidarScan, 1, 0.25);
            lidarScan.rotate(robot.heading); // Rotate scan to align with heading

            Vec2f newLidarEstimatedPosition(robot.position);
            if(robot.poseEstimator.update(lidarScan, robot.landmarks, robot.position, newLidarEstimatedPosition)) {
                Vec2f error = newLidarEstimatedPosition - robot.position;
                dpd.appendPoint(newLidarEstimatedPosition, YELLOW, NEW_ESTIMATED_POSITION_POINT);

                float alpha = std::exp(-lidarDt.count() / 1000.0f / LIDAR_TAU);
                //robot.position += error * (1.0f - alpha);

                cout << "Position: " << robot.position.x << ", " << robot.position.y << " m, Heading: " << robot.heading / M_PI * 180 << " degrees" << endl;
            }
            else {
                cout << "Pose estimation failed, keeping previous estimate." << endl;
            }
            
            robot.guidanceData.appendWaypoint(Vec2f(1, 1));

            robot.gp.update(dpd);
        }

        std::chrono::milliseconds UIDt = std::chrono::duration_cast<std::chrono::milliseconds>(now - lastUIUpdateTime);
        if (UIDt >= std::chrono::milliseconds(50)) {
            lastUIUpdateTime = now;

            robot.displayUI.position = robot.position;
            robot.displayUI.heading = robot.heading;
            robot.displayUI.update();
            dpd.updateVisibility(robot.visibility);	
        }
    }
    
    std::string name() const override {return "RunCourseState";}

	std::chrono::high_resolution_clock::time_point lastLidarUpdateTime;
    std::chrono::high_resolution_clock::time_point lastEncoderUpdateTime;
    std::chrono::high_resolution_clock::time_point lastUIUpdateTime;

};
