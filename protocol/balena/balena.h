#ifndef __BALENA_H_INCLUDED__
#define __BALENA_H_INCLUDED__

#include <platform/emlib/inc/em_cmu.h>
#include <platform/emlib/inc/em_emu.h>
#include <platform/emlib/inc/em_rtcc.h>
#include <platform/emlib/inc/em_chip.h>
#include "init_mcu.h"
//#include "init_board.h"
//#include "init_app.h"
#include "em_gpio.h"
#include "em_adc.h"
#include "em_idac.h"
#include "em_chip.h"
#include "ble-configuration.h"
#include "board_features.h"
#include "hal-config.h"
#include <platform/emdrv/ustimer/inc/ustimer.h>
#include <platform/emdrv/tempdrv/inc/tempdrv.h>
#include <platform/emdrv/rtcdrv/rtcdriver.h>

#if defined(HAL_CONFIG)
#include "bsphalconfig.h"
#else
#include "bspconfig.h"
#endif

#ifdef __cplusplus
extern "C" {
#endif

#define EFR32BGM111 1
#define NUM_PINS 20

#define DAC_COUNT 6

#define HIGH 1
#define LOW  0


/* balenaFin Pin Modes */
#define MODE_NONE        0
#define MODE_DIGITAL     1
#define MODE_ANALOG_IN   2
#define MODE_ANALOG_OUT  3
#define MODE_I2C         4
#define MODE_SPI         5

/* balenaFin Digital Pin Modes */
#define INPUT_PULLUP gpioModeInputPull

/* ADC */
#define adcFreq 13000000
typedef unsigned char byte;

/* Pin Struct */
struct PIN_MAP {
	GPIO_Port_TypeDef port;
	unsigned int pin;
	ADC_PosSel_TypeDef adc;
	IDAC_OutMode_TypeDef dac;
	unsigned short state;
};

/* Setup BGM111 */

void balenaInit();
void initMCU();
void initMCU_CLK();
void initGPIO();
void initTimer();
void initIDAC();
void initADC();

#define DEVINFO_MODULEINFO_HFXOCALVAL_MASK  0x00080000UL
// Calibration value for HFXO CTUNE is at DEVINFO Offset 0x08
#define DEVINFO_MODULEINFO_CRYSTALOSCCALVAL  (*((uint16_t *) (uint32_t)(DEVINFO_BASE+0x8UL)))
// [15:9] : (LFXOTUNING) Calibration for LFXO TUNING
// [8:0]  : (HFXOCTUNE) Calibration for HFXO CTUNE
#define DEVINFO_HFXOCTUNE_MASK  0x01FFUL

/* Arduino Functions */

void delay(unsigned int n); // delay for n milliseconds
void pinMode(unsigned int pin_no, unsigned int mode);

void digitalWrite(unsigned int pin_no, unsigned int value);
unsigned int digitalRead(unsigned int pin_no);

void analogWrite(unsigned int pin_no, byte value);

/* BalenaFin Functions */

uint32_t millis();

/* Analog Methods */
void setADC(unsigned int pin_no, ADC_TypeDef * adc);
void setIDAC(unsigned int pin_no, IDAC_TypeDef * dac);

uint32_t analogRead(unsigned int pin_no);

/* GPIO Methods */
void deviceMode(unsigned int pin_no, unsigned int mode);

/* I2C Methods */

/* SPI Methods */


#ifdef __cplusplus
}
#endif

#endif
