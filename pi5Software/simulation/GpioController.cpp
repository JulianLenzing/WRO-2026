#include "GpioController.h"

GpioController::GpioController() {
}

GpioController::~GpioController(){
}

void GpioController::setLed1High() {return;}

void GpioController::setLed2High() {return;}

int GpioController::queryButton() {
    return 1;
}

void GpioController::enableSdaSwitch(){
    return;
}

void GpioController::disableSdaSwitch(){
    return;
}