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

class EncoderController { 
public:
    GpioController& gpioController; 
    float lastAngleLeft = 0;
    float lastAngleRight = 0;
    float lastDistanceLeft = 0;
    float lastDistanceRight = 0;

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
        if(!grabData(lastAngleLeft, lastAngleRight)) printf("Failed to grab initial encoder data\n");
    }

    ~EncoderController() {
        if(fdOpen) {
            close(fd);
        }
    }

    int reset() {
        if(!grabData(lastAngleLeft, lastAngleRight)) {printf("Failed to grab encoder data to reset\n"); return 0;}
        float lastDistanceLeft = 0;
        float lastDistanceRight = 0;
        return 1;
    }

    int getEncodingData(float& deltaDistance, float& deltaHeading) {
        float angleLeft = 0;
        float angleRight = 0;
        if(!grabData(angleLeft, angleRight)) return 0;
        
        // Handle wrap around and calculate revolutions
        float deltaAngleLeft = angleLeft - lastAngleLeft;
        if(deltaAngleLeft > M_PI) deltaAngleLeft -= 2.0f * M_PI;
        else if(deltaAngleLeft < -M_PI) deltaAngleLeft += 2.0f * M_PI;
        float deltaAngleRight = angleRight - lastAngleRight;
        if(deltaAngleRight > M_PI) deltaAngleRight -= 2.0f * M_PI;
        else if(deltaAngleRight < -M_PI) deltaAngleRight += 2.0f * M_PI;
        
        float deltaDistanceLeft = WHEEL_CIRCUMFERENCE * deltaAngleLeft / (2.0f * M_PI);
        float deltaDistanceRight = WHEEL_CIRCUMFERENCE * deltaAngleRight / (2.0f * M_PI);
        
        deltaDistance = (deltaDistanceLeft + deltaDistanceRight) * 0.5f;
        deltaHeading = (deltaDistanceRight - deltaDistanceLeft) / WHEEL_DISTANCE;

        // --- EMA smoothing ---
        static float smoothedDeltaDistance = 0.0f;
        static float smoothedDeltaHeading  = 0.0f;
        const float alphaDistance = 0.2f; // 0.0-1.0, smaller = smoother
        const float alphaHeading  = 0.2f; // 0.0-1.0, smaller = smoother

        smoothedDeltaDistance = alphaDistance * deltaDistance + (1.0f - alphaDistance) * smoothedDeltaDistance;
        smoothedDeltaHeading  = alphaHeading  * deltaHeading  + (1.0f - alphaHeading)  * smoothedDeltaHeading;

        // --- Return smoothed values ---
        deltaDistance = smoothedDeltaDistance;
        deltaHeading  = smoothedDeltaHeading;

        lastAngleLeft = angleLeft;
        lastAngleRight = angleRight;
        lastDistanceLeft = deltaDistanceLeft;
        lastDistanceRight = deltaDistanceRight;
        return 1;
    }

    int fd;
    bool fdOpen = false;

    static float normaliseAngle(float angle) {
        return fmodf(fmodf(angle, 2.0f * M_PI) + 2.0f * M_PI, 2.0f * M_PI); 
    }

private:
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

        angle = float(rawAngle) * 2.0f * M_PI / 4096.0f;
        angle = normaliseAngle(angle);
        return 1;
    }

    int grabData(float& angleLeft, float& angleRight){
        float angleLeft1 = 0.0f;
        float angleLeft2 = 0.0f;
		gpioController.disableSdaSwitch();
		if(!dumbGrabData(angleLeft1)) return 0;
		gpioController.enableSdaSwitch();	
		if(!dumbGrabData(angleRight)) return 0;
        gpioController.disableSdaSwitch();
		if(!dumbGrabData(angleLeft2)) return 0;
		angleLeft = atan2f(
            sinf(angleLeft1) + sinf(angleLeft2),
            cosf(angleLeft1) + cosf(angleLeft2)
        ); // Average the two readings to reduce bias; Wrap around exists so this complicated form is needed

		angleRight = 2.0f * M_PI - angleRight;
		return 1;
    }
};