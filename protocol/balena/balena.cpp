#include "balena.h"

/* Balena Functions */

BalenaClass::BalenaClass()
{
	initMcu();
	initBoard();
	initApp();
	initTimer();
	CMU_ClockEnable(cmuClock_GPIO, true);
}

void BalenaClass::initTimer(){
	USTIMER_Init();
};

void BalenaClass::initADC(){
	CMU_ClockEnable(cmuClock_ADC0, true);
	ADCInit = ADC_INIT_DEFAULT;
	ADCSingle = ADC_INITSINGLE_DEFAULT;
	ADCInit.prescale = ADC_PrescaleCalc(adcFreq, 0); // Init to max ADC clock for Series 0

	ADCSingle.diff       = false;        // single ended
	ADCSingle.reference  = adcRef5V;    // internal 2.5V reference
	ADCSingle.resolution = adcRes12Bit;  // 12-bit resolution
	ADCSingle.acqTime    = adcAcqTime4;     // set acquisition time to meet minimum requirement
};

void BalenaClass::setADC(unsigned int pin_no, ADC_TypeDef * adc){
	ADCSingle.posSel = port_pin[pin_no].adc;
	port_pin[pin_no].state = MODE_ANALOG_IN;
	ADC_Init(adc, &ADCInit);
	ADC_InitSingle(adc, &ADCSingle);

};

void BalenaClass::initDAC(){
	EMU_DCDCInit_TypeDef dcdcInit = EMU_DCDCINIT_DEFAULT;
	EMU_DCDCInit(&dcdcInit);
	 // Enable IDAC clock
	CMU_ClockEnable(cmuClock_IDAC0, true);

	// Initialize IDAC
	DACInit = IDAC_INIT_DEFAULT;
	IDAC_Init(IDAC0, &DACInit);


}

void BalenaClass::setDAC(unsigned int pin_no, IDAC_TypeDef * dac){
	// Choose the output current to be 2 microamps
	IDAC_RangeSet(IDAC0, idacCurrentRange1);
	IDAC_StepSet(IDAC0, 4);

	// Enable IDAC output mode and also enable the IDAC module itself
	IDAC_OutEnable(IDAC0, true);
	IDAC_Enable(IDAC0, true);
};

/* Arduino Functions */

void deviceMode(unsigned int pin, unsigned int mode){
	GPIO_PinModeSet(BSP_VCOM_ENABLE_PORT, BSP_VCOM_ENABLE_PIN, gpioModePushPull, 1);
};

void pinMode(unsigned int pin_no, unsigned int value){
	GPIO_PinModeSet(Balena.port_pin[pin_no].port, Balena.port_pin[pin_no].pin , gpioModePushPull, value);
};

bool pinExists(unsigned int pin_no){
	if(pin_no <= ((sizeof(Balena.port_pin) / sizeof(PIN_MAP)))){
		return false;
	}
	else {
		return true;
	};
};

/* Digital I/O */

unsigned int digitalRead(unsigned int pin_no){
	return GPIO_PinInGet (Balena.port_pin[pin_no].port, Balena.port_pin[pin_no].pin);
};

void digitalWrite(unsigned int pin_no, unsigned int value){
	GPIO_PinModeSet(Balena.port_pin[pin_no].port, Balena.port_pin[pin_no].pin , gpioModePushPull, value);
};

/* Analog I/O */

void analogWrite(unsigned int pin_no, byte value){
	if(Balena.port_pin[pin_no].state == MODE_ANALOG_OUT){

	}
	else {

	};
};

unsigned int analogRead(unsigned int pin_no, byte value){
	unsigned int data;
	data = ADC_DataSingleGet(ADC0);
	return data;
};

/* Timer Functions */

void delay(unsigned int n){
	USTIMER_Delay(n * 1000);
};





