#pragma once

#include "sl_lidar.h" 
#include "sl_lidar_driver.h"
#include "LidarPoint.h"

sl::ILidarDriver* initLidar();
int startLidar(sl::ILidarDriver* drv);
int getLidarScan(sl::ILidarDriver* drv, LidarScan& scan, float scale = 1.0f, float subtractor = 0);
void stopLidar(sl::ILidarDriver* drv);

class Lidar
{
public:
	Lidar(float pScale = 1.0f, float pSubtractor = 0.0f) 
		: 
		scale(pScale),
		subtractor(pSubtractor),
		driver(nullptr)
		{}
		
	bool init()
	{
		driver = initLidar();
		if(!driver) return false;
		return true;
	}
	
	bool start()
	{
		if(!driver) return false;
		if(!startLidar(driver)) return false;
		return true;
	}
	
	bool getScan(LidarScan& scan)
	{
		if(!driver) return false;
		if(!getLidarScan(driver, scan, scale, subtractor)) return false;
		return true;
	}
	
	void stop()
	{
		if(!driver) return;
		stopLidar(driver);
	}
	
	float scale;
	float subtractor;
	
private:
	sl::ILidarDriver* driver;
};
