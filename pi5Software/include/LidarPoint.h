#ifndef LIDARPOINT_H
#define LIDARPOINT_H

#include <vector>

#include "Vec2f.h"
#include "Line.h"

class LidarPoint {
    public:
    float angle; // In Radians!
    float distance;
    
    LidarPoint() : angle(0.0f), distance(0.0f){}
    LidarPoint(float pAngle, float pDistance) {
        angle = normaliseAngle(pAngle);
        distance = pDistance;
    }

    static float normaliseAngle(float pAngle) {return fmodf(float(fmodf(pAngle, float(2*M_PI))+2*M_PI), 2*M_PI);}

    [[nodiscard]] Vec2f getDirection() const {return Vec2f{cosf(angle), sinf(angle)};}

    [[nodiscard]] Vec2f point() const {
        return Vec2f(cosf(angle) * distance, sinf(angle) * distance);
    }
};

class LidarScan {
    public:
    std::vector<LidarPoint> scan;
};

#endif //LIDARPOINT_H
