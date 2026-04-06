#include "Gyro.h"

Gyro::Gyro() {
    // Capture initial yaw
    reset();
}

Gyro::~Gyro() {
}

int Gyro::reset() {
    //lastYaw = current;
    return 1;
}

int Gyro::getDeltaHeading(float& heading) {
    float raw = 0.0;
    if (!readRawYaw(raw)) return 0;

    heading = raw - lastYaw;

    // Wrap delta into (-π, π] to handle crossing 0/2π boundary
    if (heading >  M_PI) heading -= 2.0f * M_PI;
    if (heading < -M_PI) heading += 2.0f * M_PI;

    lastYaw = raw;  // advance for next call
    return 1;
}

float Gyro::normaliseAngle(float angle) {
    return fmodf(fmodf(angle, 2.0f * M_PI) + 2.0f * M_PI, 2.0f * M_PI); 
}

int Gyro::readRawYaw(float& yaw) {
    return 1;
}

