#pragma once

#include <unistd.h>
#include <cmath>
#include <algorithm>
#include <PiPCA9685/PCA9685.h>

class ServoControl {
public:
    ServoControl(int pLine, float pAngleRange, float pDutyCycleRange = 1.0f)
    : line(pLine),
    angleRange(pAngleRange),
    dutyCycleRange(pDutyCycleRange),
    pca("/dev/i2c-1", 0x40)
    {
        pca.set_pwm_freq(50.0);
    }

    ~ServoControl() {
        setMs(1.5);
    }
    
    void setAngle(float angle);
    
    void setMs(float ms){
        int off = int(ms/20.0f*4096.0f);
        pca.set_pwm(line, 0, off);
    }

private:
    int line;
    float angleRange;
    float dutyCycleRange;
    PiPCA9685::PCA9685 pca;
};

  /*
void ServoControl::setAngle(float angle) {
		float lowerBoundary = angleRange / 2.0f;
		float upperBoundary = 2.0f*M_PI - (angleRange/2.0f);
		if(angle <= 0.0f) angle = 0.0f;
		else if(angle >= 2.0f*M_PI) angle = 2.0f*M_PI;
		else {
			if(angle <= M_PI && angle > lowerBoundary) angle = lowerBoundary;
			else if(angle < upperBoundary && angle > lowerBoundary) angle = upperBoundary;
		}
		
		float f;
		if(angle <= lowerBoundary) {
			f = 1-(angle / (angleRange/2.0f));
		}
		else {
			f = (angle - (2.0f*M_PI - (angleRange/2.0f))) / (angleRange/2.0f);
		}
		float ms = f * dutyCycleRange;
		int off = int(ms/20.0f*4096.0f);
		pca.set_pwm(0, 0, off);
	}
*/
/*
void ServoControl::setAngle(float angle)
{
    float error = angle;

    // Wrap to [-?, ?]
    while (error > M_PI)  error -= 2.0f * M_PI;
    while (error < -M_PI) error += 2.0f * M_PI;

    float halfRange = angleRange / 2.0f;
    error = std::clamp(error, -halfRange, halfRange);

    float normalized = error / halfRange;   // -1 .. +1
    float pulseMs = 1.5f + normalized * (dutyCycleRange / 2.0f);

    int off = static_cast<int>(pulseMs / 20.0f * 4096.0f);
    pca.set_pwm(0, 0, off);
}
*/


	


