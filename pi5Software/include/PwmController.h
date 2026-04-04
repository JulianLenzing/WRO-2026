#pragma once

#include <unistd.h>
#include <cmath>
#include <algorithm>
#include <PiPCA9685/PCA9685.h>
#include <iostream>

class PwmController {
public:
    PwmController(int pLine, float pDutyCycleRange = 1.0f);
    virtual ~PwmController();

protected:
    void setMs(float ms);

    int line;
    float dutyCycleRange;
    PiPCA9685::PCA9685 pca;
};

class MotorController : public PwmController {
public:
    MotorController(int pLine, float pDutyCycleRange = 1.0f);
    ~MotorController();
        
    void unlockControl();
    void setThrottle(float pThrottle);
    float getThrottle() const;

private:
    float currentThrottle;
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
