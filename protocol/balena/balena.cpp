#include "balena.h"

BalenaClass::BalenaClass() : SerialObj(0,9600)
{
	initMcu();
	initBoard();
	initApp();
	initTimer();
}

void pinMode(unsigned int pin, unsigned int mode){
	GPIO_PinModeSet(BSP_VCOM_ENABLE_PORT, BSP_VCOM_ENABLE_PIN, gpioModePushPull, 1);

};


void BalenaClass::digitalWrite(unsigned int pin, unsigned int value){

};

void delay(int n){
	USTIMER_Delay(n * 1000);
};

void digitalWrite(unsigned long pin_no, unsigned int value){
	GPIO_PinModeSet(port_pin[pin_no].port, port_pin[pin_no].pin , gpioModePushPull, value);
};

unsigned int digitalRead(unsigned long pin_no){
	return GPIO_PinModeGet(port_pin[pin_no].port, port_pin[pin_no].pin);
};

void BalenaClass::initTimer(){
	USTIMER_Init();
};
