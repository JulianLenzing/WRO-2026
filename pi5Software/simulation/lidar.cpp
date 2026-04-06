#include "lidar.h"

#include <iostream>
#include <fstream>
#include <vector>
#include <thread>
#include <chrono>
#include <sys/stat.h>

#define LIDAR_PATH "/dev/shm/lidar_shm.raw"
#define NUMBER_OF_SAMPLES 500
#define LIDAR_FRAME_SIZE (NUMBER_OF_SAMPLES * 2 * sizeof(float))

// Buffer for raw data (angle, distance pairs)
static std::vector<float> lidar_raw(NUMBER_OF_SAMPLES * 2);

// Helper: check if shared memory file exists
static bool file_exists(const char* path) {
    struct stat buffer;
    return (stat(path, &buffer) == 0);
}

// Init (simulation → no real hardware)
sl::ILidarDriver* initLidar() {
    // No real driver needed for simulation
    return nullptr;
}

// Start (wait for shared memory)
int startLidar(sl::ILidarDriver* drv) {
    (void)drv; // unused

    std::cout << "[LIDAR] Waiting for shared memory: " << LIDAR_PATH << std::endl;

    while (!file_exists(LIDAR_PATH)) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    std::cout << "[LIDAR] Shared memory detected." << std::endl;
    return 1;
}

// Read one scan
int getLidarScan(sl::ILidarDriver* drv, LidarScan& scan, float scale, float offset) {
    (void)drv; // unused

    std::ifstream file(LIDAR_PATH, std::ios::binary);
    if (!file) {
        std::cerr << "[LIDAR] Failed to open shared memory file." << std::endl;
        return 0;
    }

    // Read full frame
    file.read(reinterpret_cast<char*>(lidar_raw.data()), LIDAR_FRAME_SIZE);

    if (file.gcount() != LIDAR_FRAME_SIZE) {
        std::cerr << "[LIDAR] Incomplete frame read." << std::endl;
        return 0;
    }

    // Clear previous scan data
    scan.scan.clear();
    scan.scan.reserve(NUMBER_OF_SAMPLES);

    // Convert raw data into scan points
    for (int i = 0; i < NUMBER_OF_SAMPLES; i++) {
        float angle = lidar_raw[i * 2];
        float distance = lidar_raw[i * 2 + 1];

        // Skip invalid measurements
        if (distance <= 0.001f)
            continue;

        // Apply scaling + offset
        distance = distance * scale - offset;

        scan.scan.emplace_back(angle, distance);
    }

    return 1;
}

// Stop (nothing to clean up in simulation)
void stopLidar(sl::ILidarDriver* drv) {
    (void)drv; // unused
    std::cout << "[LIDAR] Stopped." << std::endl;
}