#include "balena.h"

void initBalena()
{
	initMcu();
	initBoard();
	initApp();
	GPIO_PinOutClear(BSP_VCOM_ENABLE_PORT, BSP_VCOM_ENABLE_PIN);
}


