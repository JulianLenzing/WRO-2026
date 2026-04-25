#pragma once

#include <vector>

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
    Environment() {
        float outerLength = 3.0f;
        float innerLength = 1.0f;
        outerBottomLeft = Vec2f(0, 0);
        innerBottomLeft = Vec2f(1.0f, 1.0f);
        outerTopRight = Vec2f(outerLength, outerLength);        
        innerTopRight = innerBottomLeft + Vec2f(innerLength, innerLength);
        middle = outerBottomLeft + Vec2f(outerLength, outerLength) * 0.5f;

        landmarks.emplace_back(Landmark(outerBottomLeft, Vec2f(0, outerLength)));
        landmarks.emplace_back(Landmark(Vec2f(0, outerLength), Vec2f(outerLength, outerLength)));
        landmarks.emplace_back(Landmark(Vec2f(outerLength, outerLength), Vec2f(outerLength, 0)));
        landmarks.emplace_back(Landmark(Vec2f(outerLength, 0), Vec2f(0, 0)));
        landmarks.emplace_back(Landmark(innerBottomLeft, innerBottomLeft + Vec2f(0, innerLength)));
        landmarks.emplace_back(Landmark(innerBottomLeft + Vec2f(0, innerLength), innerTopRight));
        landmarks.emplace_back(Landmark(innerTopRight, innerBottomLeft + Vec2f(innerLength, 0)));
        landmarks.emplace_back(Landmark(innerBottomLeft + Vec2f(innerLength, 0), innerBottomLeft));
    }
    
    std::vector<Landmark> landmarks;
    Vec2f outerBottomLeft;
    Vec2f outerTopRight;
    Vec2f innerBottomLeft;
    Vec2f innerTopRight;
    Vec2f middle;
};