#include "balena.h"

/* Balena Functions */

/* Analog Variables */
ADC_Init_TypeDef ADCInit;
ADC_InitSingle_TypeDef ADCSingle;
bool ADCset = false;

IDAC_Init_TypeDef DACInit;
IDAC_OutMode_TypeDef * IDACNone;

/* Pin Definitions */
PIN_MAP port_pin[NUM_PINS] = {
	{gpioPortA, 0, adcPosSelAPORT3XCH8, idacOutputAPORT1YCH9, MODE_NONE},
	{gpioPortA, 1, adcPosSelAPORT4XCH9, * IDACNone, MODE_NONE},
	{gpioPortA, 2, adcPosSelAPORT4YCH10, * IDACNone, MODE_NONE},
	{gpioPortF, 6U,adcPosSelAPORT2YCH22, * IDACNone, MODE_NONE}, // LED on BGM11 Dev Kit
	{gpioPortF, 0, adcPosSelAPORT1XCH16, * IDACNone, MODE_NONE},
	{gpioPortB, 11, adcPosSelAPORT3YCH27, * IDACNone, MODE_NONE}

	// Finish the rest of Pin Definitions
};

void balenaInit()
{
	CHIP_Init();
//	initMcu();
//	initBoard();
//	initApp();
	initTimer();
	initADC();
	CMU_ClockEnable(cmuClock_GPIO, true);
}

void initTimer(){
	USTIMER_Init();
};

void initADC(){
	CMU_ClockEnable(cmuClock_ADC0, true);
	ADCInit = ADC_INIT_DEFAULT;
	ADCSingle = ADC_INITSINGLE_DEFAULT;
	ADCInit.prescale = ADC_PrescaleCalc(adcFreq, 0); // Init to max ADC clock for Series 0

	ADCSingle.diff       = false;        // single ended
	ADCSingle.reference  = adcRef5V;    // internal 2.5V reference
	ADCSingle.resolution = adcRes12Bit;  // 12-bit resolution
	ADCSingle.acqTime    = adcAcqTime4;     // set acquisition time to meet minimum requirement
};

void setADC(unsigned int pin_no, ADC_TypeDef * adc){
	ADCSingle.posSel = port_pin[pin_no].adc;
	port_pin[pin_no].state = MODE_ANALOG_IN;
	ADC_Init(adc, &ADCInit);
	ADC_InitSingle(adc, &ADCSingle);
};

void initDAC(){
	EMU_DCDCInit_TypeDef dcdcInit = EMU_DCDCINIT_DEFAULT;
	EMU_DCDCInit(&dcdcInit);
	 // Enable IDAC clock
	CMU_ClockEnable(cmuClock_IDAC0, true);

	// Initialize IDAC
	DACInit = IDAC_INIT_DEFAULT;
	IDAC_Init(IDAC0, &DACInit);
}

void setDAC(unsigned int pin_no, IDAC_TypeDef * dac){
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
	GPIO_PinModeSet(port_pin[pin_no].port, port_pin[pin_no].pin , gpioModePushPull, value);
};

bool pinExists(unsigned int pin_no){
	if(pin_no <= ((sizeof(port_pin) / sizeof(PIN_MAP)))){
		return false;
	}
	else {
		return true;
	};
};

/* Digital I/O */

unsigned int digitalRead(unsigned int pin_no){
	return GPIO_PinInGet (port_pin[pin_no].port, port_pin[pin_no].pin);
};

void digitalWrite(unsigned int pin_no, unsigned int value){
	GPIO_PinModeSet(port_pin[pin_no].port, port_pin[pin_no].pin , gpioModePushPull, value);
};

/* Analog I/O */

void analogWrite(unsigned int pin_no, byte value){
	if(port_pin[pin_no].state == MODE_ANALOG_OUT){

	}
	else {

	};
};

uint32_t analogRead(unsigned int pin_no){
	if(port_pin[pin_no].state != MODE_ANALOG_IN){
		setADC(pin_no, ADC0);
//		digitalWrite(3,0);
	}
	 ADC_Start(ADC0, adcStartSingle);
	 while(!(ADC0->STATUS & _ADC_STATUS_SINGLEDV_MASK));
	return  ADC_DataSingleGet(ADC0);
};

/* Timer Functions */

void delay(unsigned int n){
	USTIMER_Delay(n * 1000);
};



