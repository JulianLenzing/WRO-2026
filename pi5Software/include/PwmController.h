#pragma once

#include <unistd.h>
#include <cmath>
#include <algorithm>
#include <PiPCA9685/PCA9685.h>
#include <iostream>

class PwmController {
public:
    PwmController(int pLine, float pDutyCycleRange = 1.0f)
    : line(pLine),
    dutyCycleRange(pDutyCycleRange),
    pca("/dev/i2c-1", 0x40)
    {
        pca.set_pwm_freq(50.0);
    }

    virtual ~PwmController() {
        setMs(1.5);
    }

protected:
    void setMs(float ms){
        ms = std::clamp(ms, 1.5f - dutyCycleRange / 2.0f, 1.5f + dutyCycleRange / 2.0f);
        int off = int(ms/20.0f*4096.0f);
        pca.set_pwm(line, 0, off);
    }

    int line;
    float dutyCycleRange;
    PiPCA9685::PCA9685 pca;
};

class MotorController : PwmController {
public:
    MotorController(int pLine, float pDutyCycleRange = 1.0f) 
        : PwmController(pLine, pDutyCycleRange) {}
        
    void unlockControl() { setMs(1.5); }
    void setSpeed(float speed) {
        speed = std::clamp(speed, -1.0f, 1.0f);
        float ms = 1.5f + speed * (dutyCycleRange / 2.0f);
        setMs(ms);
    } 
};

class ServoController : PwmController {
public:
    ServoController(int pLine, float pAngleRange, float pDutyCycleRange = 1.0f) // Angles in radians
        : PwmController(pLine, pDutyCycleRange), 
          angleRange(pAngleRange),
          minDutyCycle(1.5f - (pDutyCycleRange / 2.0f)),
          maxDutyCycle(1.5f + (pDutyCycleRange / 2.0f)),
          minAngle(2.0f*M_PI - (pAngleRange/2.0f)),
          maxAngle(pAngleRange/2.0f)
    {
        setMs(1.5);
    }

    void setAngle(float angle) {
        if(inverted) angle = 2.0f*M_PI - angle;
        angle = normaliseAngle(angle);
        if(angle <= M_PI && angle > maxAngle) {
            angle = maxAngle;            
        }
        else if(angle > M_PI && angle < minAngle) {
            angle = minAngle;
        }
        
        if(angle <= M_PI) {
            float ms = 1.5f - (angle / (angleRange/2.0f)) * (dutyCycleRange / 2.0f);
            ms = std::clamp(ms, minDutyCycle, maxDutyCycle);
            setMs(ms); 
        }
        else {
            float inverseAngle = 2.0f*M_PI - angle;
            float ms = 1.5f + (inverseAngle / (angleRange/2.0f)) * (dutyCycleRange / 2.0f);
            ms = std::clamp(ms, minDutyCycle, maxDutyCycle);
            setMs(ms);            
        }
    } 

    void setMiddle() { setAngle(0.0f); }

    void invert(){
        if(!inverted) inverted = true;
        else inverted = false;
    }

protected:
    float normaliseAngle(float angle) {
        float ret = fmod(fmod(angle, 2.0f*M_PI) + 2.0f*M_PI, 2.0f*M_PI);
        return ret;
    }

    bool inverted = false;

    float angleRange;
    float minDutyCycle;
    float maxDutyCycle;
    float minAngle;
    float maxAngle;
};

  /*
void ServoController::setAngle(float angle) {
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
void ServoController::setAngle(float angle)
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


	


