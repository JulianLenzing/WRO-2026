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
    Obstacle() : position(0.0f, 0.0f), color(OBSTACLE_COLOR::OBSTACLE_COLOUR_UNKNOWN), count(0) {}
    Obstacle(Vec2f pPosition, OBSTACLE_COLOR pColor = OBSTACLE_COLOR::OBSTACLE_COLOUR_UNKNOWN, size_t pCount = 0) : position(pPosition), color(pColor), count(pCount) {}
    ~Obstacle() {}

    Vec2f position;
    OBSTACLE_COLOR color;
    size_t count;
};
