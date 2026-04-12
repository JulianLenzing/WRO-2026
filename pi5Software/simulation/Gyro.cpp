#include "Gyro.h"

#include <fcntl.h>
#include <unistd.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <cstdio>
#include <cmath>

static float* g_sharedYaw = nullptr;
static int fd = -1;

bool initSharedYaw() {
    const char* path = "/dev/shm/gyro_shm.raw";

    // Open shared file (must already be created by Godot)
    fd = open(path, O_RDONLY);
    if (fd == -1) {
        perror("open gyro shared file");
        return false;
    }

    // Check file size (must be at least 4 bytes)
    struct stat st;
    if (fstat(fd, &st) == -1) {
        perror("fstat");
        close(fd);
        fd = -1;
        return false;
    }

    if (st.st_size < (off_t)sizeof(float)) {
        printf("Gyro shared file too small (%ld bytes)\n", st.st_size);
        close(fd);
        fd = -1;
        return false;
    }

    // Map file into memory
    void* ptr = mmap(
        nullptr,
        sizeof(float),
        PROT_READ,
        MAP_SHARED,
        fd,
        0
    );

    if (ptr == MAP_FAILED) {
        perror("mmap");
        close(fd);
        fd = -1;
        return false;
    }

    g_sharedYaw = static_cast<float*>(ptr);

    // fd can be closed after mmap
    close(fd);
    fd = -1;

    printf("Gyro: shared memory mapped successfully\n");
    return true;
}

void cleanupSharedYaw() {
    if (g_sharedYaw) {
        munmap(g_sharedYaw, sizeof(float));
        g_sharedYaw = nullptr;
    }
}

Gyro::Gyro() {
    if (!initSharedYaw()) {
        printf("Gyro init failed\n");
    }
    reset();
}

Gyro::~Gyro() {
    cleanupSharedYaw();
}

int Gyro::reset() {
    float current = 0.0f;

    if (!readRawYaw(current)) {
        printf("Gyro: failed to read yaw for reset\n");
        return 0;
    }

    lastYaw = current;
    return 1;
}

int Gyro::readRawYaw(float& yaw) {
    if (!g_sharedYaw) {
        printf("Gyro: shared memory not initialized\n");
        return 0;
    }

    yaw = *g_sharedYaw;
    return 1;
}

int Gyro::getDeltaHeading(float& heading) {
    float raw = 0.0f;

    if (!readRawYaw(raw)) {
        return 0;
    }

    float diff = raw - lastYaw;

    // shortest signed angular difference (-pi, pi]
    diff = atan2f(sinf(diff), cosf(diff));

    heading = diff;
    lastYaw = raw;

    //printf("Gyro: raw=%.3f delta=%.3f\n", raw, heading);

    return 1;
}

float Gyro::normaliseAngle(float angle) {
    return fmodf(fmodf(angle, 2.0f * M_PI) + 2.0f * M_PI, 2.0f * M_PI);
}