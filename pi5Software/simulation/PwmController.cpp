// File: RobotControllersSim.cpp
#include "PwmController.h"
#include <cmath>
#include <algorithm>
#include <cstring>
#include <iostream>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>

// =======================
// Shared Memory Helper
// =======================
class ShmWriter {
private:
    const char* shm_name;
    int shm_fd = -1;
    float* data = nullptr;
    size_t num_floats;

public:
    ShmWriter(const char* name, size_t n)
        : shm_name(name), num_floats(n)
    {
        shm_fd = shm_open(shm_name, O_CREAT | O_RDWR, 0666);
        if(shm_fd < 0) { perror("shm_open"); return; }

        ftruncate(shm_fd, sizeof(float) * num_floats);
        data = (float*)mmap(0, sizeof(float) * num_floats,
                             PROT_WRITE | PROT_READ, MAP_SHARED, shm_fd, 0);
        if(data == MAP_FAILED) { perror("mmap"); data = nullptr; }
        if(data) std::memset(data, 0, sizeof(float) * num_floats);
    }

    ~ShmWriter() {
        if(data) munmap(data, sizeof(float) * num_floats);
        if(shm_fd >= 0) close(shm_fd);
        shm_unlink(shm_name);
    }

    void write(size_t index, float value) {
        if(data && index < num_floats) {
            data[index] = value;
            msync(data, sizeof(float) * 2, MS_SYNC); // force sync
        }
    }
};

// =======================
// Global shared memory
// =======================
ShmWriter globalShm("/control.raw", 2); // index 0=steer, 1=throttle

// =======================
// PwmController Implementation
// =======================
PwmController::PwmController(int pLine, float pDutyCycleRange)
    : line(pLine), dutyCycleRange(pDutyCycleRange)
{
    // Simulation: do nothing
}

PwmController::~PwmController() {}

void PwmController::setMs(float ms) {
    // Simulation: do nothing
}

// =======================
// MotorController Implementation
// =======================
MotorController::MotorController(int pLine, float pDutyCycleRange)
    : PwmController(pLine, pDutyCycleRange), currentThrottle(0.0f)
{
    setThrottle(0.0f);
}

MotorController::~MotorController() {
    setThrottle(0.0f);
}

void MotorController::unlockControl() { return; }

void MotorController::setThrottle(float pThrottle) {
    pThrottle = std::clamp(pThrottle, -1.0f, 1.0f);
    currentThrottle = pThrottle;

    // Write to shared memory (index 1 = throttle)
    globalShm.write(1, currentThrottle);
}

float MotorController::getThrottle() const { return currentThrottle; }

// =======================
// ServoController Implementation
// =======================
ServoController::ServoController(int pLine, float pAngleRange, float pDutyCycleRange)
    : PwmController(pLine, pDutyCycleRange),
      angleRange(pAngleRange),
      minDutyCycle(1.5f - pDutyCycleRange/2.0f),
      maxDutyCycle(1.5f + pDutyCycleRange/2.0f),
      minAngle(2.0f*M_PI - angleRange/2.0f),
      maxAngle(angleRange/2.0f),
      inverted(false),
      currentAngle(0.0f)
{
    setMiddle();
}

ServoController::~ServoController() {
    setMiddle();
}

void ServoController::setAngle(float angle) {
    currentAngle = angle;

    if(inverted) angle = 2.0f*M_PI - angle;
    angle = normaliseAngle(angle);

    if(angle <= M_PI && angle > maxAngle) angle = maxAngle;
    else if(angle > M_PI && angle < minAngle) angle = minAngle;

    // Write to shared memory (index 0 = steer)
    globalShm.write(0, angle);
}

float ServoController::getAngle() const { return currentAngle; }

void ServoController::setMiddle() { setAngle(0.0f); }

void ServoController::invert() { inverted = !inverted; }

float ServoController::normaliseAngle(float angle) {
    return std::fmod(std::fmod(angle, 2.0f*M_PI) + 2.0f*M_PI, 2.0f*M_PI);
}