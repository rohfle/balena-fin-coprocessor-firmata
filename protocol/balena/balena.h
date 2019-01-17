#ifndef __BALENA_H_INCLUDED__
#define __BALENA_H_INCLUDED__

#include <platform/emlib/inc/em_cmu.h>
#include <platform/emlib/inc/em_emu.h>
#include "init_mcu.h"
#include "init_board.h"
#include "init_app.h"
#include "em_gpio.h"
#include "ble-configuration.h"
#include "board_features.h"
#include "hal-config.h"
#include <platform/emdrv/ustimer/inc/ustimer.h>
#include "Serial.h"

#if defined(HAL_CONFIG)
#include "bsphalconfig.h"
#else
#include "bspconfig.h"
#endif

#ifdef __cplusplus
extern "C" {
#endif

#define EFR32BGM111 1

#define HIGH 1
#define LOW 0


const struct PIN_MAP {
	GPIO_Port_TypeDef port;
	unsigned int pin;
} port_pin[] = {
	{gpioPortA,0},
	{gpioPortA,1},
	{gpioPortA,2}
	// Finish the rest of Pin Definitions
};

void delay(unsigned long n); // delay for n milliseconds
void digitalWrite(unsigned int pin, unsigned int value);
unsigned int digitalRead(unsigned int pin);


class BalenaClass{

private:
	SerialClass SerialObj;
	void initTimer();

public:
	BalenaClass();

	void digitalWrite(unsigned int pin, unsigned int value);
	void pinMode(unsigned int pin, unsigned int mode);


};

#ifdef __cplusplus
}
#endif

#endif
