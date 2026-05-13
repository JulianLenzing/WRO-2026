#pragma once

#include <vector>

#include "Run_Type.h"
#include "Line.h"
#include "Vec2f.h"

class Landmark
{
public:
    explicit Landmark(Line pLine, bool pIsUseable = true) : line(pLine), isUseable(pIsUseable) {}
    explicit Landmark(Vec2f a, Vec2f b, bool pIsUseable = true) : line(a, b), isUseable(pIsUseable) {}


    Line line;
    bool isUseable;
};

class Environment {
public:
    Environment(const float& robotLength, const RUN_TYPE& runType, bool parkingObstacle) {
        float outerLength = 3.0f;
        float innerLength = 1.0f;
        outerBottomLeft = Vec2f(0, 0);
        innerBottomLeft = Vec2f(1.0f, 1.0f);
        outerTopRight = Vec2f(outerLength, outerLength);        
        innerTopRight = innerBottomLeft + Vec2f(innerLength, innerLength);
        middle = outerBottomLeft + Vec2f(outerLength, outerLength) * 0.5f;
        bool innerBoundariesAreUseable = true;
        float parkingObstacleLength = robotLength * 1.5f;

        if (runType == RUN_TYPE_OPENING_RUN)
        {
            innerBoundariesAreUseable = false;
            innerBottomLeft = Vec2f(0.6f, 0.6f);
            innerTopRight = Vec2f(2.4f, 2.4f);
            innerLength = 1.8f;
        }

        landmarks.emplace_back(Landmark(outerBottomLeft, Vec2f(0, outerLength), true));
        landmarks.emplace_back(Landmark(Vec2f(0, outerLength), Vec2f(outerLength, outerLength), true));
        landmarks.emplace_back(Landmark(Vec2f(outerLength, outerLength), Vec2f(outerLength, 0), true));        

        if(!parkingObstacle) landmarks.emplace_back(Landmark(Vec2f(outerLength, 0), Vec2f(0, 0), true));
        else 
        {
            landmarks.emplace_back(Landmark(outerBottomLeft, Vec2f(1.98f - parkingObstacleLength, 0.0f), true));
            landmarks.emplace_back(Landmark(Vec2f(2.0f - parkingObstacleLength, 0.0f), Vec2f(2.0f, 0.0f), false));
            landmarks.emplace_back(Landmark(Vec2f(2.0f, 0.0f), Vec2f(outerLength, 0.0f), true));            
        }

        landmarks.emplace_back(Landmark(innerBottomLeft, innerBottomLeft + Vec2f(0, innerLength), innerBoundariesAreUseable));
        landmarks.emplace_back(Landmark(innerBottomLeft + Vec2f(0, innerLength), innerTopRight, innerBoundariesAreUseable));
        landmarks.emplace_back(Landmark(innerTopRight, innerBottomLeft + Vec2f(innerLength, 0), innerBoundariesAreUseable));
        landmarks.emplace_back(Landmark(innerBottomLeft + Vec2f(innerLength, 0), innerBottomLeft, innerBoundariesAreUseable));
        
        if(parkingObstacle)
        {        
            landmarks.emplace_back(Landmark(Vec2f(2.0f, 0.0f), Vec2f(1.98f, 0.2f), false));
            landmarks.emplace_back(Landmark(Vec2f(2.0f - parkingObstacleLength, 0.0f), Vec2f(2.0f - parkingObstacleLength, 0.2f), false));
        }
    }
    
    std::vector<Landmark> landmarks;
    Vec2f outerBottomLeft;
    Vec2f outerTopRight;
    Vec2f innerBottomLeft;
    Vec2f innerTopRight;
    Vec2f middle;
};
