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

    GpioController();
    ~GpioController();

    void setLed1High();
    void setLed2High();
    int queryButton();
    void enableSdaSwitch();
    void disableSdaSwitch();
};
