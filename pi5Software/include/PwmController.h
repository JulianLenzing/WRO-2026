#pragma once

#include <unistd.h>
#include <cmath>
#include <algorithm>
#include <iostream>
#include <chrono>

#ifndef SIMULATION 
#include <PiPCA9685/PCA9685.h> 
#endif

class PwmController {
public:
    PwmController(int pLine, float pDutyCycleRange = 1.0f);
    virtual ~PwmController();

protected:
    void setMs(float ms);

    int line;
    float dutyCycleRange;
    #ifndef SIMULATION 
    PiPCA9685::PCA9685 pca; 
    #endif
};

class MotorController : public PwmController {
public:
    MotorController(int pLine, float pDutyCycleRange = 1.0f);
    ~MotorController();
        
    void setThrottle(float pThrottle);
    float getThrottle() const;

private:
    std::chrono::high_resolution_clock::time_point timerStart();
    double timerEnd(std::chrono::high_resolution_clock::time_point start);

    float currentThrottle;
    float lastNonZeroThrottle;
    std::chrono::high_resolution_clock::time_point directionChangeTime;
    bool directionChange;
};

class ServoController : public PwmController {
public:
    ServoController(int pLine, float pAngleRange, float pDutyCycleRange = 1.0f); // Angles in radians
    ~ServoController();

    void setAngle(float angle); 
    float getAngle() const;
    void setMiddle();
    void invert();

protected:
    static float normaliseAngle(float angle);

    bool inverted;
    float currentAngle;
    float angleRange;
    float minDutyCycle;
    float maxDutyCycle;
    float minAngle;
    float maxAngle;
};
