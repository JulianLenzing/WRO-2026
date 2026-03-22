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

#define WHEEL_CIRCUMFERENCE 	0.135f
#define WHEEL_DISTANCE		0.102f
#define LOWER_REVOLUTION_LIMIT 	90.0f
#define UPPER_REVOLUTION_LIMIT 	270.0f

#define AS5600_ADDR 0x36

class EncoderController { 
public:
    GpioController& gpioController; 
    float angleLeftStart, angleRightStart;	
    int revolutionsLeft = 0;
    int revolutionsRight = 0;
    float lastAngleLeft = 0;
    float lastAngleRight = 0;
    float lastDistance = 0;

public:
    EncoderController(GpioController& pGpioController) 
        :   gpioController(pGpioController)
    {
        fd = open("/dev/i2c-1", O_RDWR);
        if (fd < 0) {
            perror("open failed");
            return;
        }
        fdOpen = true;
        if(!grabData(angleLeftStart, angleRightStart)) printf("Failed to grab initial encoder data\n");
    }

    ~EncoderController() {
        if(fdOpen) {
            close(fd);
        }
    }

    int getEncodingData(float& deltaDistance, float& heading) {
        float angleLeft, angleRight;
        if(!grabData(angleLeft, angleRight)) return 0;
        angleLeft = normaliseAngle(angleLeft - angleLeftStart);
        angleRight = normaliseAngle(angleRight - angleRightStart);
        
        if(lastAngleLeft > UPPER_REVOLUTION_LIMIT && angleLeft < LOWER_REVOLUTION_LIMIT) revolutionsLeft++;
        if(lastAngleLeft < LOWER_REVOLUTION_LIMIT && angleLeft > UPPER_REVOLUTION_LIMIT) revolutionsLeft--;
        
        if(lastAngleRight > UPPER_REVOLUTION_LIMIT && angleRight < LOWER_REVOLUTION_LIMIT) revolutionsRight++;
        if(lastAngleRight < LOWER_REVOLUTION_LIMIT && angleRight > UPPER_REVOLUTION_LIMIT) revolutionsRight--;
        
        float distanceLeft = WHEEL_CIRCUMFERENCE * (revolutionsLeft + angleLeft/360.0f);
        float distanceRight = WHEEL_CIRCUMFERENCE * (revolutionsRight + angleRight/360.0f);
        float distance = (distanceLeft + distanceRight) / 2;
        
        heading = (distanceRight - distanceLeft) / (WHEEL_DISTANCE * 2 * M_PI) * 360.0f;
        heading = normaliseAngle(heading);
        deltaDistance = distance - lastDistance;
        
        // Fix for this not being in radians
        heading = heading * M_PI / 180.0f;

        lastAngleLeft = angleLeft;
        lastAngleRight = angleRight;
        lastDistance = distance;
        return 1;
    }

    int fd;
    bool fdOpen = false;

private:
    float normaliseAngle(float angle){
        return fmodf(fmodf(angle, 360.0f) + 360.0f, 360.0f); 
    }

    int dumbGrabData(float& angle){
        if(!fdOpen) return 0;

        if (ioctl(fd, I2C_SLAVE, AS5600_ADDR) < 0) {
            perror("ioctl failed");
            close(fd);
            fdOpen = false;
            return 0;
        }

        unsigned char buffer[2] = {0};

        int ret = i2c_smbus_read_i2c_block_data(fd, 0x0C, 2, buffer);
        //printf("ret: %d\n", ret);

        if(ret != 2) {
            perror("i2c read failed");
            close(fd);
            fdOpen = false;
            return 0;
        }

        short rawAngle = buffer[0] << 8 | buffer[1];
        rawAngle &= 0x0FFF;

        angle = float(rawAngle) * 360.0f / 4096.0f;

        return 1;
    }

    int grabData(float& angleLeft, float& angleRight){
		gpioController.disableSdaSwitch();
		if(!dumbGrabData(angleLeft)) return 0;
		gpioController.enableSdaSwitch();	
		if(!dumbGrabData(angleRight)) return 0;
		angleRight = 360.0f - angleRight;
		return 1;
    }
};