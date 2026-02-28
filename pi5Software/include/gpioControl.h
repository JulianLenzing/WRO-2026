#define LED1 			6
#define LED2 			5
#define BUTTON			4
#define SDA_SWITCH		16
#define SCL_SWITCH		17

int initGpioControl();
void queryButton();
void enableSdaSwitch();
void disableSdaSwitch();
void deInitGpioControl();
