#pragma once

class RGBA {
public:
    RGBA(){}
    RGBA(float pR, float pG, float pB, float pA) : r(pR), g(pG), b(pB), a(pA) {}
    float r{1}, g{1}, b{1}, a{1};
};
