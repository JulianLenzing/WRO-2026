#pragma once

#include <iostream>
#include <thread>
#include <mutex>
#include <queue>
#include <optional>

#include "Vec2f.h"

class GuidanceData
{
    private:
    std::mutex mtx;
    std::queue<Vec2f> waypoints{};
    bool runGuidance = false;
    bool runThread = true;

    Vec2f position;
    float heading;

    float steeringAngle;
    float throttle;
    Vec2f currentWaypoint;

    public:
    GuidanceData() : position(0.0f, 0.0f), heading(0.0f), steeringAngle(0.0f), throttle(0.0f), currentWaypoint(-1, -1) {}

    void start() {
        std::lock_guard<std::mutex> lock(mtx);
        runGuidance = true;
    }
    
    void stop() {
        std::lock_guard<std::mutex> lock(mtx);
        runGuidance = false;
    }

    void setRobotData(Vec2f pos, float head) {
        std::lock_guard<std::mutex> lock(mtx);
        position = pos;
        heading = head;
    }

    void getRobotData(Vec2f& pos, float& head) {
        std::lock_guard<std::mutex> lock(mtx);
        pos = position;
        head = heading;
    }

    void setUiData(float pSteeringAngle, float pThrottle) {
        std::lock_guard<std::mutex> lock(mtx);
        steeringAngle = pSteeringAngle;
        throttle = pThrottle;
    }

    void getUiData(float& pSteeringAngle, float& pThrottle) {
        std::lock_guard<std::mutex> lock(mtx);
        pSteeringAngle = steeringAngle;
        pThrottle = throttle;
    }
    
    bool getGuidanceStatus() {
        std::lock_guard<std::mutex> lock(mtx);
        return runGuidance;
    }
    
    void terminate() {
        std::lock_guard<std::mutex> lock(mtx);
        runThread = false;
    }
    
    bool getThreadStatus() {
        std::lock_guard<std::mutex> lock(mtx);
        return runThread;
    }
    
    void appendWaypoint(Vec2f waypoint)
    {
        std::lock_guard<std::mutex> lock(mtx);
        waypoints.push(waypoint);
    }

    std::optional<Vec2f> getWaypoint()
    {
        std::lock_guard<std::mutex> lock(mtx);
        if (waypoints.empty())
        {
            return std::nullopt;
        }
        std::optional<Vec2f> optWaypoint = waypoints.front();
        waypoints.pop();
        currentWaypoint = optWaypoint.value();
        return optWaypoint;
    }

    Vec2f lookAtCurrentWaypoint() {
        return currentWaypoint;
    }

    size_t getWaypointCount()
    {
        std::lock_guard<std::mutex> lock(mtx);
        return waypoints.size();
    }
};
