#pragma once

#include <vector>

#include "Line.h"
#include "Vec2f.h"

class Landmarks {
public:
    Landmarks() {
        float outerLength = 3.0f;
        float innerLength = 1.0f;
        outerBottomLeft = Vec2f(0, 0);
        innerBottomLeft = Vec2f(1.0f, 1.0f);
        outerTopRight = Vec2f(outerLength, outerLength);        
        innerTopRight = innerBottomLeft + Vec2f(innerLength, innerLength);
        middle = outerBottomLeft + Vec2f(outerLength, outerLength) * 0.5f;

        lines.emplace_back(outerBottomLeft, Vec2f(0, outerLength));
        lines.emplace_back(Vec2f(0, outerLength), Vec2f(outerLength, outerLength));
        lines.emplace_back(Vec2f(outerLength, outerLength), Vec2f(outerLength, 0));
        lines.emplace_back(Vec2f(outerLength, 0), Vec2f(0, 0));
        lines.emplace_back(innerBottomLeft, innerBottomLeft + Vec2f(0, innerLength));
        lines.emplace_back(innerBottomLeft + Vec2f(0, innerLength), innerTopRight);
        lines.emplace_back(innerTopRight, innerBottomLeft + Vec2f(innerLength, 0));
        lines.emplace_back(innerBottomLeft + Vec2f(innerLength, 0), innerBottomLeft);
    }
    
    std::vector<Line> lines;
    Vec2f outerBottomLeft;
    Vec2f outerTopRight;
    Vec2f innerBottomLeft;
    Vec2f innerTopRight;
    Vec2f middle;
};