#ifndef LIDAR_H
#define LIDAR_H

#include "sl_lidar.h" 
#include "sl_lidar_driver.h"
#include "LidarPoint.h"

int startLidar(sl::ILidarDriver* drv);
int getLidarScan(sl::ILidarDriver* drv, LidarScan& scan, float scale = 1.0f, float subtractor = 0);
void stopLidar(sl::ILidarDriver* drv);

#endif
