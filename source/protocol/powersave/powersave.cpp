#include "powersave.h"

/*
 * Powersave SYSEX Firmata command
 * Request:
 *   0xF0        start SYSEX
 *   0x0F        SYSEX command ID for powersave - user-defined
 *   0x00-0x7F   delay before cutting power in seconds
 *   0x00-0x7F   sleep time in minutes (LSB)
 *   0x00-0x7F   sleep time in minutes (MSB)
 *   0xF7        end SYSEX
 * Response:
 *   0xF0        start SYSEX
 *   0x0F        SYSEX command ID for powersave - user-defined
 *   0xF7        end SYSEX
 */

#define GPIO_PORT_3V3_EN        gpioPortF
#define GPIO_PIN_3V3_EN         5

#define GPIO_PORT_5V_EN         gpioPortC
#define GPIO_PIN_5V_EN          9


enum PowersaveFSM {S0_POWERON, S1_SHUTTING_DOWN, S2_POWEROFF};

uint32_t nowtime;
uint32_t prevtime;

uint32_t shutdown_delay;
uint32_t startup_delay;

PowersaveFSM state;

void powersaveInit() {
  state = S0_POWERON;
  GPIO_PinModeSet(GPIO_PORT_5V_EN, GPIO_PIN_5V_EN, gpioModeWiredAnd, 1);
  GPIO_PinModeSet(GPIO_PORT_3V3_EN, GPIO_PIN_3V3_EN, gpioModeWiredAnd, 1);
  GPIO_PinOutSet(GPIO_PORT_5V_EN, GPIO_PIN_5V_EN);
  GPIO_PinOutSet(GPIO_PORT_3V3_EN, GPIO_PIN_3V3_EN);
}

void powersaveSet(uint8_t shutdown_secs, uint16_t startup_mins) {
  shutdown_delay = shutdown_secs;
  startup_delay = startup_mins * 60;
  state = S1_SHUTTING_DOWN;
  prevtime = RTCDRV_GetWallClock();
}

void powersaveRun() {
  if (state == S0_POWERON) {
    // do nothing
  } else if (state == S1_SHUTTING_DOWN) {
    nowtime = RTCDRV_GetWallClock();
    if ((nowtime - prevtime) > shutdown_delay) {
      GPIO_PinOutClear(GPIO_PORT_3V3_EN, GPIO_PIN_3V3_EN);
      RTCDRV_Delay(200);
      GPIO_PinOutClear(GPIO_PORT_5V_EN, GPIO_PIN_5V_EN);
      state = S2_POWEROFF;
      prevtime = nowtime;
    }
  } else if (state == S2_POWEROFF) {
    nowtime = RTCDRV_GetWallClock();
    if ((nowtime - prevtime) > startup_delay) {
      GPIO_PinOutSet(GPIO_PORT_5V_EN, GPIO_PIN_5V_EN);
      RTCDRV_Delay(200);
      GPIO_PinOutSet(GPIO_PORT_3V3_EN, GPIO_PIN_3V3_EN);
      state = S0_POWERON;
      prevtime = 0;
    }
  }
}
