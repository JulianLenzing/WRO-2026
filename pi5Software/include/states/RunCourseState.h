#pragma once

#include "State.h"
#include "RobotSystem.h"

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
    void update(RobotSystem& robot) override
    {
		dpd.clear();
		LidarScan lidarScan;
		getLidarScan(robot.lidarDriver, lidarScan, 1, 0.25);
		//for(auto p : lidarScan.scan) {printf("%.2f %.2f\n", p.distance, p.angle);}
		writeLidarScanToFile(lidarScan, "LidarTestData");
		Vec2f newEstimatedPosition(robot.estimatedPosition);
		doPoseEstimation(lidarScan, robot.estimatedPosition, 0.1f, newEstimatedPosition);
		robot.estimatedPosition = robot.estimatedPosition * 0.8 + newEstimatedPosition * 0.2;
		robot.guidanceData.appendWaypoint(Vec2f(1, 1));
		
		robot.displayUI.update();
		dpd.updateVisibility(robot.visibility);	
		robot.gp.update(dpd);
    }
    
    std::string name() const override {return "RunCourseState";}
};
