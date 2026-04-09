#pragma once

#include <optional>
#include <vector>

#include "Vec2f.h"
#include "Obstacle.h"
#include "LidarPoint.h"

#include "DisplayData.h"

#define OBSTACLE_DETECTION_RADIUS 0.05f
#define OBSTACLE_DETECTION_COUNT size_t(30)

class ObstacleDetection
{
public:
    ObstacleDetection()
    {
        std::vector<Obstacle> basePoints = {
            {Obstacle(Vec2f(1.0f, 0.4f), 4)},
            {Obstacle(Vec2f(1.0f, 0.6f), 1)},
            {Obstacle(Vec2f(1.5f, 0.4f), 5)},
            {Obstacle(Vec2f(1.5f, 0.6f), 2)},
            {Obstacle(Vec2f(2.0f, 0.4f), 6)},
            {Obstacle(Vec2f(2.0f, 0.6f), 3)}
        };

        Vec2f pivot(1.5f, 1.5f);

        for (const auto& p : basePoints) {
            Obstacle current = p;

            for (int i = 0; i < 4; ++i) {
                possibleObstacles.emplace_back(current);
                current.position = rotate90(current.position, pivot);
            }
        }
    }

    ~ObstacleDetection() {}

    void feedScan(LidarScan scan, Vec2f estimatedPosition)
    {
        float radiusSquared = OBSTACLE_DETECTION_RADIUS * OBSTACLE_DETECTION_RADIUS;
        for (LidarPoint point : scan.scan)
        {
            for (Obstacle& possibleObstacle : possibleObstacles)
            {
                if (isInRadiusSquared(point.point() + estimatedPosition, possibleObstacle.position, radiusSquared))
                {
                    if (possibleObstacle.count < 999999) possibleObstacle.count++; // Safety against overflow
                    dpd.appendPoint(point.point() + estimatedPosition, WHITE);
                }
            }
        }
    }

    void getObstacles(std::vector<Obstacle>& obstacles)
    {
        for(Obstacle o : possibleObstacles) {
            if(o.count >= OBSTACLE_DETECTION_COUNT) obstacles.push_back(o);
        }        
    }

    std::vector<Obstacle> possibleObstacles;

protected:
    bool isInRadiusSquared(Vec2f positionA, Vec2f positionB, float radiusSquared)
    {
        float distanceSquared = Vec2f(positionB-positionA).lengthSquared();
        if (distanceSquared < radiusSquared) return true;
        return false;
    }

    Vec2f rotate90(const Vec2f& p, const Vec2f& center) {
        float x = p.x - center.x;
        float y = p.y - center.y;

        float rx = -y;
        float ry = x;

        return Vec2f(rx + center.x, ry + center.y);
    }
};
