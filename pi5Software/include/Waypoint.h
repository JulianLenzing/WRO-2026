#pragma once

#include "Vec2f.h"

class Waypoint {
public:
    Waypoint() : point(0.0f, 0.0f), heading(0.0f), reverse(false), slow(true), closingDistance(0.5f), maxThrottle(1.0f) {}
    Waypoint(Vec2f pPoint, float pHeading, bool pSlow = true, bool pReverse = false, float pClosingDistance = 0.4f, float pMaxThrottle = 1.0f) : point(pPoint), heading(fmodf(fmodf(pHeading, 2.0f*M_PI) + 2.0f*M_PI, 2.0f*M_PI)), slow(pSlow), reverse(pReverse), closingDistance(pClosingDistance), maxThrottle(pMaxThrottle) {}

    Vec2f point;
    float heading;
    bool reverse;
    bool slow;
    float closingDistance;
    float maxThrottle;
};
