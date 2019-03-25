
#include <time.h>
#include <unistd.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

enum PowersaveFSM {S0_POWERON, S1_SHUTTING_DOWN, S2_POWEROFF};

time_t nowtime;
time_t prevtime;

PowersaveFSM state;
uint32_t shutdown_delay;
uint32_t startup_delay;

bool gpio_3v3_en;

void powersaveInit() {
  state = S0_POWERON;
  gpio_3v3_en = true;
}

void powersaveSet(uint8_t shutdown_secs, uint16_t startup_mins) {
  shutdown_delay = shutdown_secs;
  startup_delay = startup_mins;
  state = S1_SHUTTING_DOWN;
  prevtime = time(0);
}

void powersaveRun() {
  if (state == S0_POWERON) {
    // do nothing
  } else if (state == S1_SHUTTING_DOWN) {
    nowtime = time(0);
    if (difftime(nowtime, prevtime) > shutdown_delay) {
      gpio_3v3_en = false;
      // RTCDRV_Delay(200);
      // GPIO_PinOutClear(GPIO_PORT_5V_EN, GPIO_PIN_5V_EN);
      state = S2_POWEROFF;
      prevtime = nowtime;
    }
  } else if (state == S2_POWEROFF) {
    nowtime = time(0);
    if (difftime(nowtime, prevtime) > startup_delay) {
      // GPIO_PinOutSet(GPIO_PORT_5V_EN, GPIO_PIN_5V_EN);
      // RTCDRV_Delay(200);
      gpio_3v3_en = true;
      state = S0_POWERON;
      prevtime = 0;
    }
  }
}


int main() {
  powersaveInit();

  time_t last_time = time(0) - 120;
  time_t now_time;

  bool beforegpio;

  PowersaveFSM beforestate;
  int secs_until_shutdown = 1;
  int mins_until_startup = 1;

  while (1) {

    if (difftime(time(0), last_time) > 10) {
      powersaveSet(secs_until_shutdown, mins_until_startup);
      printf("Setting powersave - shutdown in %d seconds, then %d seconds before start\n",
             secs_until_shutdown, mins_until_startup);
      last_time = time(0);
    }

    beforestate = state;
    beforegpio = gpio_3v3_en;
    powersaveRun();
    printf("Run - state(%d -> %d) gpio(%d -> %d)\n",
           beforestate, state, beforegpio, gpio_3v3_en);
    sleep(1);
  }
}
