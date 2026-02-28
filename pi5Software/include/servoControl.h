#pragma once

#include <cmath>
#include <PiPCA9685/PCA9685.h>

class ServoControl {
public:
    explicit ServoControl(
        float pAngleRange = M_PI / 2.0f,
        float pDutyCycleRange = 1.0f
    );

    void setAngle(float angle);
    void setMs(float ms = 1.5);

private:
    float angleRange;
    float dutyCycleRange;
    PiPCA9685::PCA9685 pca;
};
