#pragma once

#include <gpiod.h>

#define LED1 			6
#define LED2 			5
#define BUTTON			4
#define SDA_SWITCH		16
#define SCL_SWITCH		17

struct gpiod_line_request* request_output_line(const char *chip_path, unsigned int offset, enum gpiod_line_value value);
struct gpiod_line_request* request_input_line(const char *chip_path, unsigned int offset);
const char * gpiod_value_str(enum gpiod_line_value value);
enum gpiod_line_value toggle_line_value(enum gpiod_line_value value);