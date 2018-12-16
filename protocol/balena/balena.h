#ifndef __BALENA_H_INCLUDED__
#define __BALENA_H_INCLUDED__

#include <platform/emlib/inc/em_cmu.h>
#include <platform/emlib/inc/em_emu.h>
#include "init_mcu.h"
#include "init_board.h"
#include "init_app.h"
#include "ble-configuration.h"
#include "board_features.h"
#include "hal-config.h"
//#include "Serial.h"

#if defined(HAL_CONFIG)
#include "bsphalconfig.h"
#else
#include "bspconfig.h"
#endif

#ifdef __cplusplus
extern "C" {
#endif

#define EFR32BG111

void initBalena();

#ifdef __cplusplus
}
#endif

#endif
