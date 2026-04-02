#pragma once

#include "Vec2f.h"

class Waypoint {
public:
    Waypoint() : point(0.0f, 0.0f), reverse(false) {}
    Waypoint(Vec2f pPoint, bool pReverse = false) : point(pPoint), reverse(pReverse) {}

    Vec2f point;
    bool reverse;
};
