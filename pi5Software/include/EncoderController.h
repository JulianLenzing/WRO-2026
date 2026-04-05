#pragma once

#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
extern "C" {
#include <i2c/smbus.h>
}
#include <chrono>
#include <cmath>
#include <cstdio>

#include "GpioController.h"

#define WHEEL_CIRCUMFERENCE 	0.13274f
#define WHEEL_DISTANCE		0.09863f

#define AS5600_ADDR 0x36
#define MAX_GRAB_RETRIES 3

class EncoderController { 
public:
    GpioController& gpioController; 
    float lastAngleLeft = 0;
    float lastAngleRight = 0;
    float lastDistanceLeft = 0;
    float lastDistanceRight = 0;

    explicit EncoderController(GpioController& pGpioController);
    ~EncoderController();

    int reset();
    int getEncodingData(float& deltaDistance, float& deltaHeading);
    static float normaliseAngle(float angle);
    
private:    
    int fd;
    bool fdUseable = false;

    int dumbGrabData(float& angle);
    int grabData(float& angleLeft, float& angleRight);
    int retryingGrabData(float& angle);
    void busyWaitMicroseconds(int us);
};