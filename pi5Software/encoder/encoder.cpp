#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
extern "C" {
#include <i2c/smbus.h>
}
#include <chrono>
#include "gpioControl.h"

#define AS5600_ADDR 0x36

int dumbGrabData(float& angle){
	int fd = open("/dev/i2c-1", O_RDWR);
	ioctl(fd, I2C_SLAVE, AS5600_ADDR);
	unsigned char buffer[2];
	for(auto& c : buffer) {c = 0x00;}
	int ret = i2c_smbus_read_i2c_block_data(fd, 0x0C, 2, buffer);
	if(ret != 2) return 0;
	short rawAngle = buffer[0] << 8 | buffer[1];
	rawAngle = rawAngle & 0b0000111111111111;
	angle = float(rawAngle) * 360.0f / 4096.0f;
	return 1;
}

int grabData(float& angleLeft, float& angleRight){
		disableSdaSwitch();
		if(!dumbGrabData(angleLeft)) return 0;
		enableSdaSwitch();	
		if(!dumbGrabData(angleRight)) return 0;
		angleRight = 360.0f - angleRight;
		return 1;
}
