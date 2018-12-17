#include "balena.h"


using namespace balena;

void BalenaClass::init()
{
	initMcu();
	initBoard();
	initApp();
	GPIO_PinOutClear(BSP_VCOM_ENABLE_PORT, BSP_VCOM_ENABLE_PIN);
}


