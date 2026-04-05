#include "GpioController.h"

GpioController::GpioController() {
    static const char *const chip_path = "/dev/gpiochip0";    

    led1_request = request_output_line(chip_path, LED1, GPIOD_LINE_VALUE_INACTIVE);
    led2_request = request_output_line(chip_path, LED2, GPIOD_LINE_VALUE_INACTIVE);
    sda_switch_request = request_output_line(chip_path, SDA_SWITCH, GPIOD_LINE_VALUE_INACTIVE);
    scl_switch_request = request_output_line(chip_path, SCL_SWITCH, GPIOD_LINE_VALUE_INACTIVE);
    
    button_request = request_input_line(chip_path, BUTTON);
    
}

GpioController::~GpioController(){
    if(led1_request)        {gpiod_line_request_set_value(led1_request, LED1, GPIOD_LINE_VALUE_INACTIVE); gpiod_line_request_release(led1_request);}
    if(led2_request)        {gpiod_line_request_set_value(led2_request, LED2, GPIOD_LINE_VALUE_INACTIVE); gpiod_line_request_release(led2_request);}
    if(sda_switch_request)  gpiod_line_request_release(sda_switch_request);
    if(scl_switch_request)  gpiod_line_request_release(scl_switch_request);
    if(button_request)      gpiod_line_request_release(button_request);
}

void GpioController::setLed1High() {
   gpiod_line_request_set_value(led1_request, LED1, GPIOD_LINE_VALUE_ACTIVE);
}

void GpioController::setLed2High() {
   gpiod_line_request_set_value(led2_request, LED2, GPIOD_LINE_VALUE_ACTIVE);
}

int GpioController::queryButton() {
    enum gpiod_line_value value = gpiod_line_request_get_value(button_request, BUTTON);
    return value == GPIOD_LINE_VALUE_ACTIVE ? 0 : 1;
}

void GpioController::enableSdaSwitch(){
    gpiod_line_request_set_value(sda_switch_request, SDA_SWITCH, GPIOD_LINE_VALUE_ACTIVE);
}

void GpioController::disableSdaSwitch(){
    gpiod_line_request_set_value(sda_switch_request, SDA_SWITCH, GPIOD_LINE_VALUE_INACTIVE);
}