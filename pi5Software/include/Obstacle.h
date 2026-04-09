#pragma once

#include "Vec2f.h"

enum OBSTACLE_COLOR
{
    OBSTACLE_COLOUR_RED = 0,
    OBSTACLE_COLOUR_GREEN = 1,
    OBSTACLE_COLOUR_UNKNOWN = 2
};

class Obstacle
{
public:
    Obstacle() : position(0.0f, 0.0f), positionNumber(-1), color(OBSTACLE_COLOR::OBSTACLE_COLOUR_UNKNOWN), count(0) {}
    Obstacle(Vec2f pPosition, size_t pPositionNumber, OBSTACLE_COLOR pColor = OBSTACLE_COLOR::OBSTACLE_COLOUR_UNKNOWN, size_t pCount = 0) : position(pPosition), positionNumber(pPositionNumber), color(pColor), count(pCount) {}
    ~Obstacle() {}

    Vec2f position;
    size_t positionNumber; // The relative possible positions per side are numbered to make pathfinding easier
    // If the bottom side is observed and the robot moves in an anti clockwise direction the inner obstacles are numbered 1, 2, 3
    // and the outer obstacles are numbered 4, 5, 6 from left to right
    OBSTACLE_COLOR color;
    size_t count;
};
