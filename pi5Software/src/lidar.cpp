/*
 *  SLAMTEC LIDAR
 *  Simple Data Grabber Demo App
 *
 *  Copyright (c) 2009 - 2014 RoboPeak Team
 *  http://www.robopeak.com
 *  Copyright (c) 2014 - 2020 Shanghai Slamtec Co., Ltd.
 *  http://www.slamtec.com
 *
 */
/*
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <vector>
#include <cmath>

#include "sl_lidar.h" 
#include "sl_lidar_driver.h"
#include "LidarPoint.h"

#ifndef _countof
#define _countof(_Array) (int)(sizeof(_Array) / sizeof(_Array[0]))
#endif

static inline void delay_ms(sl_word_size_t ms){
    while (ms>=1000){
        usleep(1000*1000);
        ms-=1000;
    };
    if (ms!=0)
        usleep(ms*1000);
}

using namespace sl;

int startLidar(ILidarDriver * drv) {
	const char *opt_channel = NULL;
    const char *opt_channel_param_first = "/dev/ttyAMA0";
    sl_u32      opt_channel_param_second = 460800;
    sl_result   op_result;
	int         opt_channel_type = CHANNEL_TYPE_SERIALPORT;

    IChannel* _channel;

    sl_lidar_response_device_health_t healthinfo;
    sl_lidar_response_device_info_t devinfo;
		_channel = (*createSerialPortChannel(opt_channel_param_first, opt_channel_param_second));
        
        if (SL_IS_FAIL((drv)->connect(_channel))) {
			fprintf(stderr, "Error, cannot bind to the specified serial port %s.\n", opt_channel_param_first);		
        }

        // retrieving the device info
        ////////////////////////////////////////
        op_result = drv->getDeviceInfo(devinfo);

        if (SL_IS_FAIL(op_result)) {
            if (op_result == SL_RESULT_OPERATION_TIMEOUT) {
                // you can check the detailed failure reason
                fprintf(stderr, "Error, operation time out.\n");
            } else {
                fprintf(stderr, "Error, unexpected error, code: %x\n", op_result);
                // other unexpected result
            }
            return 0;
        }

        // print out the device serial number, firmware and hardware version number..
        printf("SLAMTEC LIDAR S/N: ");
        for (int pos = 0; pos < 16 ;++pos) {
            printf("%02X", devinfo.serialnum[pos]);
        }

        printf("\n"
                "Version:  %s \n"
                "Firmware Ver: %d.%02d\n"
                "Hardware Rev: %d\n"
                , "SL_LIDAR_SDK_VERSION"
                , devinfo.firmware_version>>8
                , devinfo.firmware_version & 0xFF
                , (int)devinfo.hardware_version);


        // check the device health
        ////////////////////////////////////////
        op_result = drv->getHealth(healthinfo);
        if (SL_IS_OK(op_result)) { // the macro IS_OK is the preperred way to judge whether the operation is succeed.
            printf("Lidar health status : ");
            switch (healthinfo.status) 
			{
				case SL_LIDAR_STATUS_OK:
					printf("OK.");
					break;
				case SL_LIDAR_STATUS_WARNING:
					printf("Warning.");
					break;
				case SL_LIDAR_STATUS_ERROR:
					printf("Error.");
					return 0;
            }
            printf(" (errorcode: %d)\n", healthinfo.error_code);

        } else {
            fprintf(stderr, "Error, cannot retrieve the lidar health code: %x\n", op_result);
            return 0;
        }


        if (healthinfo.status == SL_LIDAR_STATUS_ERROR) {
            fprintf(stderr, "Error, slamtec lidar internal error detected. Please reboot the device to retry.\n");
            // enable the following code if you want slamtec lidar to be reboot by software
            // drv->reset();
            return 0;
        }

		drv->setMotorSpeed();
		
		if (SL_IS_FAIL(drv->startScan( 0,1 ))) // you can force slamtec lidar to perform scan operation regardless whether the motor is rotating
        {
            fprintf(stderr, "Error, cannot start the scan operation.\n");
            return 0;
        }

		delay_ms(3000);
		
		return 1;
}

int getLidarScan(ILidarDriver * drv, LidarScan& scan, float scale, float subtractor) {
	sl_result ans;
    
	sl_lidar_response_measurement_node_hq_t nodes[700];
	size_t   count = _countof(nodes);

	//printf("waiting for data...\n");

	ans = drv->grabScanDataHq(nodes, count, 0);
	if (SL_IS_OK(ans) || ans == SL_RESULT_OPERATION_TIMEOUT) {
		drv->ascendScanData(nodes, count);
	} else {
		printf("error code: %x\n", ans);
		fprintf(stderr, "Error, cannot grab scan data. Could also be timeout; missing handling implementation\n");
		return 0;
	}
	
	for (int pos = 0; pos < (int)count ; ++pos) {
		/*
		printf("%s theta: %03.2f Dist: %08.2f \n", 
		(nodes[pos].flag & SL_LIDAR_RESP_HQ_FLAG_SYNCBIT) ?"S ":"  ", 
		(nodes[pos].angle_z_q14 * 90.f) / 16384.f,
		nodes[pos].dist_mm_q2/4.0f);
		*/			
		if(nodes[pos].dist_mm_q2 != 0.0f) {
			float angle = (nodes[pos].angle_z_q14 * 90.0f) / 16384.0f;
			angle = (angle / 180.0f * M_PI) + M_PI*1.5;	
			angle = fmodf(angle, 2*M_PI);
			angle = 2*M_PI - angle;
			angle = fmodf(angle, 2*M_PI);
			float distance = ((nodes[pos].dist_mm_q2 / 4.0f / 1000.0f) * scale) - subtractor;		
			if (distance > 0) scan.scan.emplace_back(angle, distance);
		}			
	}
	
	return 1;
}

void stopLidar(ILidarDriver* drv) {
    drv->stop();
	delay_ms(20);
	drv->setMotorSpeed(0);		
    if(drv) {
        delete drv;
        drv = NULL;
    }
}
