#pragma once

#include <opencv2/opencv.hpp>
#include <cstdio>
#include <string>

class Camera
{
public:
    Camera();
    ~Camera();

    cv::Mat grabFrame();
    bool isOpened() const;

private:
    #ifndef SIMULATION
    cv::VideoCapture cap;
    #endif
};
