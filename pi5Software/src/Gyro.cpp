#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
extern "C" {
#include <i2c/smbus.h>
}

#include "Gyro.h"

Gyro::Gyro() {
    fd = open("/dev/i2c-1", O_RDWR);
    if (fd < 0) {
        perror("Gyro: open failed");
        return;
    }
    fdOpen = true;

    // Capture initial yaw
    reset();
}

Gyro::~Gyro() {
    if (fdOpen) {
        close(fd);
    }
}

int Gyro::reset() {
    float current = 0.0;
    if (!readRawYaw(current)) {
        printf("Gyro: failed to grab yaw for reset\n");
        return 0;
    }
    lastYaw = current;
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
    if (!fdOpen) return 0;

    if (ioctl(fd, I2C_SLAVE, WT901_ADDR) < 0) {
        perror("Gyro: ioctl failed");
        close(fd);
        fdOpen = false;
        return 0;
    }

    unsigned char buffer[2] = {0};
    int ret = i2c_smbus_read_i2c_block_data(fd, WT901_YAW_REG, 2, buffer);
    if (ret != 2) {
        perror("Gyro: i2c read failed");
        close(fd);
        fdOpen = false;
        return 0;
    }

    int16_t raw = (int16_t)((buffer[1] << 8) | buffer[0]);
    yaw = raw / 32768.0 * M_PI;
    yaw = normaliseAngle(yaw);
    printf("Angle: %.2f\n", yaw);
    return 1;
}

