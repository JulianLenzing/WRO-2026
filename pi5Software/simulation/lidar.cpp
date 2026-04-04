#include <iostream>
#include <fstream>
#include <vector>
#include <thread>
#include <chrono>
#include <cmath>
#include <sys/stat.h>

#include "sl_lidar.h" 
#include "sl_lidar_driver.h"
#include "LidarPoint.h"

#define LIDAR_PATH "/dev/shm/lidar_shm.raw"
#define NUMBER_OF_SAMPLES 500
#define LIDAR_FRAME_SIZE size_t(NUMBER_OF_SAMPLES * 2 * sizeof(float))

std::vector<float> lidar_raw(NUMBER_OF_SAMPLES * 2);

bool file_exists(const char* path) {
    struct stat buffer;
    return (stat(path, &buffer) == 0);
}

sl::ILidarDriver* initLidar() {
    sl::ILidarDriver* drv;
    return drv;
}

int startLidar(sl::ILidarDriver* drv){
    std::cout << "Waiting for shared memory..." << std::endl;
    while (!file_exists(LIDAR_PATH)) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    return 1;
}

int getLidarScan(sl::ILidarDriver* drv, LidarScan& scan, float scale, float offset){
    std::ifstream file(LIDAR_PATH, std::ios::binary);
    if (!file) return 0;
    file.read(reinterpret_cast<char*>(lidar_raw.data()), LIDAR_FRAME_SIZE);

    for (int i = 0; i < NUMBER_OF_SAMPLES; i++) {
        float angle = lidar_raw[i * 2];
        float distance = lidar_raw[i * 2 + 1];

        if (distance <= 0.001f) continue; // skip invalid

        scan.scan.emplace_back(angle, distance);
    }
    return 1;
}

void stopLidar(sl::ILidarDriver* drv){
    return;
}