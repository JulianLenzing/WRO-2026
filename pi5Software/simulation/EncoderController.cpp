#include "EncoderController.h"

EncoderController::EncoderController(GpioController& pGpioController) 
    :   gpioController(pGpioController)
{
    if(!grabData(lastAngleLeft, lastAngleRight)) printf("Failed to grab initial encoder data\n");
}

EncoderController::~EncoderController() {
}

int EncoderController::reset() {
    if(!grabData(lastAngleLeft, lastAngleRight)) {printf("Failed to grab encoder data to reset\n"); return 0;}
    lastDistanceLeft = 0;
    lastDistanceRight = 0;
    return 1;
}

int EncoderController::getEncodingData(float& deltaDistance, float& deltaHeading) {
    return 1;
}

float EncoderController::normaliseAngle(float angle) {
    return fmodf(fmodf(angle, 2.0f * M_PI) + 2.0f * M_PI, 2.0f * M_PI); 
}

int EncoderController::dumbGrabData(float& angle) {
    return 1;
}

int EncoderController::grabData(float& angleLeft, float& angleRight) {
    return 1;
}

int EncoderController::retryingGrabData(float& angle) {return 1;}

void EncoderController::busyWaitMicroseconds(int us) {return;}