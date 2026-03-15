#pragma once

#include <gpiod.h>

#include "gpioControl.h"

class GpioController {
public:
    struct gpiod_line_request *led1_request;
    struct gpiod_line_request *led2_request;
    struct gpiod_line_request *button_request;
    struct gpiod_line_request *sda_switch_request;
    struct gpiod_line_request *scl_switch_request;

public:
    GpioController() {
        static const char *const chip_path = "/dev/gpiochip0";    
	
        led1_request = request_output_line(chip_path, LED1, GPIOD_LINE_VALUE_ACTIVE);
        led2_request = request_output_line(chip_path, LED2, GPIOD_LINE_VALUE_INACTIVE);
        sda_switch_request = request_output_line(chip_path, SDA_SWITCH, GPIOD_LINE_VALUE_INACTIVE);
        scl_switch_request = request_output_line(chip_path, SCL_SWITCH, GPIOD_LINE_VALUE_INACTIVE);
        
        button_request = request_input_line(chip_path, BUTTON);
        
    }
    
    ~GpioController(){
        gpiod_line_request_release(led1_request);
        gpiod_line_request_release(led2_request);
        gpiod_line_request_release(sda_switch_request);
        gpiod_line_request_release(scl_switch_request);
        gpiod_line_request_release(button_request);
    }

    int queryButton() {
        enum gpiod_line_value value = gpiod_line_request_get_value(button_request, BUTTON);
        gpiod_line_request_set_value(led2_request, LED2, toggle_line_value(value));
        return value == GPIOD_LINE_VALUE_ACTIVE ? 0 : 1;
    }

    void enableSdaSwitch(){
       gpiod_line_request_set_value(sda_switch_request, SDA_SWITCH, GPIOD_LINE_VALUE_ACTIVE);
    }
    
    void disableSdaSwitch(){
        gpiod_line_request_set_value(sda_switch_request, SDA_SWITCH, GPIOD_LINE_VALUE_INACTIVE);
    }
};
