#include <errno.h>
#include <gpiod.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "gpioControl.h"

struct gpiod_line_request *led1_request;
struct gpiod_line_request *led2_request;
struct gpiod_line_request *button_request;
struct gpiod_line_request *sda_switch_request;
struct gpiod_line_request *scl_switch_request;

static struct gpiod_line_request* request_output_line(const char *chip_path, unsigned int offset, enum gpiod_line_value value) {
	struct gpiod_request_config *req_cfg = NULL;
	struct gpiod_line_request *request = NULL;
	struct gpiod_line_settings *settings;
	struct gpiod_line_config *line_cfg;
	struct gpiod_chip *chip;
	int ret;

	chip = gpiod_chip_open(chip_path);
	if (!chip)
		return NULL;

	settings = gpiod_line_settings_new();
	if (!settings)
		goto close_chip;

	gpiod_line_settings_set_direction(settings, GPIOD_LINE_DIRECTION_OUTPUT);
	gpiod_line_settings_set_output_value(settings, value);

	line_cfg = gpiod_line_config_new();
	if (!line_cfg)
		goto free_settings;

	ret = gpiod_line_config_add_line_settings(line_cfg, &offset, 1, settings);
	if (ret)
		goto free_line_config;

	req_cfg = gpiod_request_config_new();
	if (!req_cfg)
		goto free_line_config;

	gpiod_request_config_set_consumer(req_cfg, "gpioControl");    

	request = gpiod_chip_request_lines(chip, req_cfg, line_cfg);

	gpiod_request_config_free(req_cfg);

free_line_config:
	gpiod_line_config_free(line_cfg);

free_settings:
	gpiod_line_settings_free(settings);

close_chip:
	gpiod_chip_close(chip);

	if(!request) fprintf(stderr, "failed to request line with offset %d: %s\n", offset, strerror(errno));
	return request;
}

static struct gpiod_line_request* request_input_line(const char *chip_path, unsigned int offset) {
	struct gpiod_request_config *req_cfg = NULL;
	struct gpiod_line_request *request = NULL;
	struct gpiod_line_settings *settings;
	struct gpiod_line_config *line_cfg;
	struct gpiod_chip *chip;
	int ret;

	chip = gpiod_chip_open(chip_path);
	if (!chip)
		return NULL;

	settings = gpiod_line_settings_new();
	if (!settings)
		goto close_chip;

	gpiod_line_settings_set_direction(settings, GPIOD_LINE_DIRECTION_INPUT);

	line_cfg = gpiod_line_config_new();
	if (!line_cfg)
		goto free_settings;

	ret = gpiod_line_config_add_line_settings(line_cfg, &offset, 1, settings);
	if (ret)
		goto free_line_config;
	
	req_cfg = gpiod_request_config_new();
	if (!req_cfg)
		goto free_line_config;

	gpiod_request_config_set_consumer(req_cfg, "gpioControl");


	request = gpiod_chip_request_lines(chip, req_cfg, line_cfg);

	gpiod_request_config_free(req_cfg);

free_line_config:
	gpiod_line_config_free(line_cfg);

free_settings:
	gpiod_line_settings_free(settings);

close_chip:
	gpiod_chip_close(chip);

	return request;
}

static const char * value_str(enum gpiod_line_value value)
{
	if (value == GPIOD_LINE_VALUE_ACTIVE)
		return "Active";
	else if (value == GPIOD_LINE_VALUE_INACTIVE) {
		return "Inactive";
	} else {
		return "Unknown";
	}
}

int initGpioControl(void) {
	static const char *const chip_path = "/dev/gpiochip0";    
	
	led1_request = request_output_line(chip_path, LED1, GPIOD_LINE_VALUE_ACTIVE);
	led2_request = request_output_line(chip_path, LED2, GPIOD_LINE_VALUE_INACTIVE);
	sda_switch_request = request_output_line(chip_path, SDA_SWITCH, GPIOD_LINE_VALUE_INACTIVE);
	scl_switch_request = request_output_line(chip_path, SCL_SWITCH, GPIOD_LINE_VALUE_INACTIVE);
	
	button_request = request_input_line(chip_path, BUTTON);
	
	return 1;
}

static enum gpiod_line_value toggle_line_value(enum gpiod_line_value value)
{
	return (value == GPIOD_LINE_VALUE_ACTIVE) ? GPIOD_LINE_VALUE_INACTIVE :
						    GPIOD_LINE_VALUE_ACTIVE;
}

void queryButton() {
	enum gpiod_line_value value = gpiod_line_request_get_value(button_request, BUTTON);
	gpiod_line_request_set_value(led2_request, LED2, toggle_line_value(value));
}

void enableSdaSwitch(){
  gpiod_line_request_set_value(sda_switch_request, SDA_SWITCH, GPIOD_LINE_VALUE_ACTIVE);
}

void disableSdaSwitch(){
  gpiod_line_request_set_value(sda_switch_request, SDA_SWITCH, GPIOD_LINE_VALUE_INACTIVE);
}

void deInitGpioControl(){
      gpiod_line_request_release(led1_request);
      gpiod_line_request_release(led2_request);
      gpiod_line_request_release(sda_switch_request);
      gpiod_line_request_release(scl_switch_request);
      gpiod_line_request_release(button_request);
}

