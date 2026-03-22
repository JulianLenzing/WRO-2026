#pragma once

#include <vector>

#include "Line.h"
#include "Vec2f.h"

class Landmarks {
public:
    Landmarks() {
        lines.emplace_back(Vec2f(0,0), Vec2f(0, 3));
        lines.emplace_back(Vec2f(0, 3), Vec2f(3, 3));
        lines.emplace_back(Vec2f(3, 3), Vec2f(3, 0));
        lines.emplace_back(Vec2f(3, 0), Vec2f(0, 0));
        lines.emplace_back(Vec2f(1, 1), Vec2f(1, 2));
        lines.emplace_back(Vec2f(1, 2), Vec2f(2, 2));
        lines.emplace_back(Vec2f(2, 2), Vec2f(2, 1));
        lines.emplace_back(Vec2f(2, 1), Vec2f(1, 1));
    }
    std::vector<Line> lines;
};