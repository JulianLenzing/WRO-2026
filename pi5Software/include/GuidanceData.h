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

    public:
    void start() {
        std::lock_guard<std::mutex> lock(mtx);
        runGuidance = true;
    }
    
    void stop() {
        std::lock_guard<std::mutex> lock(mtx);
        runGuidance = false;
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
        return optWaypoint;
    }

    size_t getWaypointCount()
    {
        std::lock_guard<std::mutex> lock(mtx);
        return waypoints.size();
    }
};
