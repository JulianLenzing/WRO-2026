#pragma once

#include "Vec2f.h"

#define OBSTACLE_DETECTION_COUNT size_t(30)
#define COLOR_DETECTION_COUNT 3

enum OBSTACLE_COLOR
{
    OBSTACLE_COLOUR_RED = 0,
    OBSTACLE_COLOUR_GREEN = 1,
    OBSTACLE_COLOUR_UNKNOWN = 2
};

class Obstacle
{
public:
    Obstacle() : position(0.0f, 0.0f), positionNumber(-1), colorCount(0), count(0) {}
    Obstacle(Vec2f pPosition, size_t pPositionNumber, size_t pCount = 0, size_t pColorCount = 0) : position(pPosition), positionNumber(pPositionNumber), colorCount(pColorCount), count(pCount) {}
    ~Obstacle() {}

    Vec2f position;
    size_t positionNumber; // The relative possible positions per side are numbered to make pathfinding easier
    // If the bottom side is observed and the robot moves in an anti clockwise direction the inner obstacles are numbered 1, 2, 3
    // and the outer obstacles are numbered 4, 5, 6 from left to right
    int colorCount; // RED -> positive GREEN -> negative
    size_t count;

    bool isValid() const
    {
        if(count >= OBSTACLE_DETECTION_COUNT) return true;
        return false;
    }

    enum OBSTACLE_COLOR getColor() const
    {
        if (colorCount > COLOR_DETECTION_COUNT) return OBSTACLE_COLOUR_RED;
        if (colorCount < -COLOR_DETECTION_COUNT) return OBSTACLE_COLOUR_GREEN;
        return OBSTACLE_COLOUR_UNKNOWN;
    }
};
