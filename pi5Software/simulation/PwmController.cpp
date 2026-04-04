#include "PwmController.h"

/*Superclass PwmController*/
// Public
PwmController::PwmController(int pLine, float pDutyCycleRange)
: line(pLine),
dutyCycleRange(pDutyCycleRange)
{
}

PwmController::~PwmController() {
}

// Protected
void PwmController::setMs(float ms){
    return;
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
    setMiddle();
}

ServoController::~ServoController(){
    setMiddle();
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
