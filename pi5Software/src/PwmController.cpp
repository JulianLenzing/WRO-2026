#include "PwmController.h"

/*Superclass PwmController*/
// Public
PwmController::PwmController(int pLine, float pDutyCycleRange)
: line(pLine),
dutyCycleRange(pDutyCycleRange),
pca("/dev/i2c-1", 0x40)
{
    pca.set_pwm_freq(50.0);
}

PwmController::~PwmController() {
    setMs(1.5);
}

// Protected
void PwmController::setMs(float ms){
    ms = std::clamp(ms, 1.5f - dutyCycleRange / 2.0f, 1.5f + dutyCycleRange / 2.0f);
    int off = int(ms/20.0f*4096.0f);
    pca.set_pwm(line, 0, off);
}

/*Subclass MotorController*/
// Public
MotorController::MotorController(int pLine, float pDutyCycleRange) 
: PwmController(pLine, pDutyCycleRange),
currentThrottle(0.0f)
{}

MotorController::~MotorController(){
    setThrottle(0.0f);
}
    
void MotorController::unlockControl() { setMs(1.5); }

void MotorController::setThrottle(float pThrottle) {
    pThrottle = std::clamp(pThrottle, -1.0f, 1.0f);
    currentThrottle = pThrottle;
    float ms = 1.5f + pThrottle * (dutyCycleRange / 2.0f);
    setMs(ms);
} 

float MotorController::getThrottle() const { return currentThrottle; }

/*Subclass ServoController*/
// Public
ServoController::ServoController(int pLine, float pAngleRange, float pDutyCycleRange) // Angles in radians
    : PwmController(pLine, pDutyCycleRange), 
    angleRange(pAngleRange),
    minDutyCycle(1.5f - (pDutyCycleRange / 2.0f)),
    maxDutyCycle(1.5f + (pDutyCycleRange / 2.0f)),
    minAngle(2.0f*M_PI - (pAngleRange/2.0f)),
    maxAngle(pAngleRange/2.0f),
    inverted(false),
    currentAngle(0.0f)
{
    setMs(1.5f);
}

ServoController::~ServoController(){
    setMs(1.5f);
}

void ServoController::setAngle(float angle) {
    currentAngle = angle;
    if(inverted) angle = 2.0f*M_PI - angle; // Invert angle if needed
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

float ServoController::getAngle() const { return currentAngle; }

void ServoController::setMiddle() { setAngle(0.0f); }


void ServoController::invert(){
    inverted = !inverted;
}

// Protected
float ServoController::normaliseAngle(float angle) {
        float ret = fmod(fmod(angle, 2.0f*M_PI) + 2.0f*M_PI, 2.0f*M_PI);
        return ret;
    }
