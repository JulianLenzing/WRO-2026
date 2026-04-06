#pragma once

#include <cstdio>
#include <cstdint>
#include <cmath>

#define WT901_ADDR 0x50
#define WT901_YAW_REG 0x3F

class Gyro {
public:
    Gyro();
    ~Gyro();

    // Returns 1 on success, 0 on failure
    int getDeltaHeading(float& heading);

    // Reset the yaw reference 
    int reset();

    // Helper function for normalising angles
    static float normaliseAngle(float angle);

private:
    int fd = -1;
    bool fdOpen = false;
    float lastYaw = 0.0;

    // Reads raw yaw from sensor, returns degrees in [-180, 180)
    int readRawYaw(float& yaw);
};