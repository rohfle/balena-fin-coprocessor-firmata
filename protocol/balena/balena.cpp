#include "balena.h"

/* Balena Functions */

/* Analog Variables */
ADC_Init_TypeDef ADCInit;
ADC_InitSingle_TypeDef ADCSingle;
ADC_PosSel_TypeDef * ADCNone;

bool ADCset = false;

IDAC_Init_TypeDef DACInit = IDAC_INIT_DEFAULT;
IDAC_OutMode_TypeDef * IDACNone;

/* RTC */
RTCDRV_TimerID_t id;

/* Pin Definitions */
PIN_MAP port_pin[NUM_PINS] = {
	{gpioPortD, 14, adcPosSelAPORT3XCH6, idacOutputAPORT1YCH9, MODE_NONE},   // P0
	{gpioPortB, 13, adcPosSelAPORT3YCH29, idacOutputAPORT1YCH29, MODE_NONE}, // P1
	{gpioPortA, 2, adcPosSelAPORT4YCH10, idacOutputAPORT1XCH10, MODE_NONE},  // P2
	{gpioPortC, 8, adcPosSelAPORT2YCH8,  * IDACNone, MODE_NONE},             // P3
	{gpioPortA, 3, adcPosSelAPORT3YCH11, idacOutputAPORT1YCH11, MODE_NONE},  // P4
	{gpioPortC, 6, adcPosSelAPORT1XCH6, * IDACNone, MODE_NONE},              // P5
	{gpioPortA, 4, adcPosSelAPORT4YCH12, idacOutputAPORT1XCH12, MODE_NONE},  // P6
	{gpioPortC, 7, adcPosSelAPORT2XCH7, * IDACNone, MODE_NONE},              // P7
	{gpioPortA, 5, adcPosSelAPORT3YCH13, idacOutputAPORT1YCH13, MODE_NONE},  // P8
	{gpioPortA, 1, adcPosSelAPORT3YCH9, idacOutputAPORT1YCH9, MODE_NONE},    // P9
	{gpioPortB, 11, adcPosSelAPORT3YCH27, idacOutputAPORT1YCH9, MODE_NONE},  // P10 <- Test ADC
	{gpioPortA, 0, adcPosSelAPORT3XCH8, idacOutputAPORT1XCH8, MODE_NONE},    // P11
	{gpioPortF, 6, adcPosSelAPORT2YCH22, * IDACNone, MODE_NONE},             // P12 (Dev Kit LED0)
	{gpioPortD, 15, adcPosSelAPORT3YCH7, idacOutputAPORT1YCH7, MODE_NONE},   // P13
	{gpioPortF, 7, adcPosSelAPORT2XCH23, * IDACNone, MODE_NONE},             // P14 (Dev Kit LED1)
	{gpioPortD, 13, adcPosSelAPORT3YCH5, idacOutputAPORT1YCH5, MODE_NONE},   // P15
	{gpioPortF, 5, * ADCNone, * IDACNone, MODE_NONE},                        // P16 (Placeholder ADC)
	{gpioPortC, 9, * ADCNone, * IDACNone, MODE_NONE},                        // P17 (Placeholder ADC)
	{gpioPortC, 10, * ADCNone, * IDACNone, MODE_NONE},                       // P18 (Placeholder ADC)
	{gpioPortC, 11, * ADCNone, * IDACNone, MODE_NONE}                        // P19 (Placeholder ADC)
};

void balenaInit()
{
	initMcu();
	initGPIO();
	initTimer();
	initADC();
}

void initMCU(){
	// Device errata
	CHIP_Init();

	// Set up DC-DC converter
	EMU_DCDCInit_TypeDef dcdcInit = BSP_DCDC_INIT;
	#if HAL_DCDC_BYPASS
	dcdcInit.dcdcMode = emuDcdcMode_Bypass;
	#endif
	EMU_DCDCInit(&dcdcInit);

	// Set up clocks
	initMCU_CLK();

	RTCC_Init_TypeDef rtccInit = RTCC_INIT_DEFAULT;
	rtccInit.enable                = true;
	rtccInit.debugRun              = false;
	rtccInit.precntWrapOnCCV0      = false;
	rtccInit.cntWrapOnCCV1         = false;
	rtccInit.prescMode             = rtccCntTickPresc;
	rtccInit.presc                 = rtccCntPresc_1;
	rtccInit.enaOSCFailDetect      = false;
	rtccInit.cntMode               = rtccCntModeNormal;
	RTCC_Init(&rtccInit);

	#if defined(_EMU_CMD_EM01VSCALE0_MASK)
	// Set up EM0, EM1 energy mode configuration
	EMU_EM01Init_TypeDef em01Init = EMU_EM01INIT_DEFAULT;
	EMU_EM01Init(&em01Init);
	#endif // _EMU_CMD_EM01VSCALE0_MASK

	#if defined(_EMU_CTRL_EM23VSCALE_MASK)
	// Set up EM2, EM3 energy mode configuration
	EMU_EM23Init_TypeDef em23init = EMU_EM23INIT_DEFAULT;
	em23init.vScaleEM23Voltage = emuVScaleEM23_LowPower;
	EMU_EM23Init(&em23init);
	#endif //_EMU_CTRL_EM23VSCALE_MASK

	TEMPDRV_Init();
}

void initMCU_CLK(void)
{
  // Initialize HFXO
  CMU_HFXOInit_TypeDef hfxoInit = BSP_CLK_HFXO_INIT;
  // if Factory Cal exists in DEVINFO then use it
  if (0==(DEVINFO->MODULEINFO & DEVINFO_MODULEINFO_HFXOCALVAL_MASK)) {
     hfxoInit.ctuneSteadyState = DEVINFO_MODULEINFO_CRYSTALOSCCALVAL & DEVINFO_HFXOCTUNE_MASK;
  }
#if defined(BSP_CLK_HFXO_CTUNE) && BSP_CLK_HFXO_CTUNE > -1
  else {
    hfxoInit.ctuneSteadyState = BSP_CLK_HFXO_CTUNE;
  }
#endif
  CMU_HFXOInit(&hfxoInit);

  // Set system HFXO frequency
  SystemHFXOClockSet(BSP_CLK_HFXO_FREQ);

  // Enable HFXO oscillator, and wait for it to be stable
  CMU_OscillatorEnable(cmuOsc_HFXO, true, true);

  // Enable HFXO Autostart only if EM2 voltage scaling is disabled.
  // In 1.0 V mode the chip does not support frequencies > 21 MHz,
  // this is why HFXO autostart is not supported in this case.
#if!defined(_EMU_CTRL_EM23VSCALE_MASK)
  // Automatically start and select HFXO
  CMU_HFXOAutostartEnable(0, true, true);
#else
  CMU_ClockSelectSet(cmuClock_HF, cmuSelect_HFXO);
#endif//_EMU_CTRL_EM23VSCALE_MASK

  // HFRCO not needed when using HFXO
  CMU_OscillatorEnable(cmuOsc_HFRCO, false, false);

  // Enabling HFBUSCLKLE clock for LE peripherals
  CMU_ClockEnable(cmuClock_HFLE, true);

  // Initialize LFXO
  CMU_LFXOInit_TypeDef lfxoInit = BSP_CLK_LFXO_INIT;
  lfxoInit.ctune = BSP_CLK_LFXO_CTUNE;
  CMU_LFXOInit(&lfxoInit);

  // Set system LFXO frequency
  SystemLFXOClockSet(BSP_CLK_LFXO_FREQ);

  // Set LFXO if selected as LFCLK
  CMU_ClockSelectSet(cmuClock_LFA, cmuSelect_LFXO);
  CMU_ClockSelectSet(cmuClock_LFB, cmuSelect_LFXO);
  CMU_ClockSelectSet(cmuClock_LFE, cmuSelect_LFXO);
}

void initGPIO(){
	CMU_ClockEnable(cmuClock_GPIO, true);
	EMU_DCDCInit_TypeDef dcdcInit = EMU_DCDCINIT_DEFAULT;
	EMU_DCDCInit(&dcdcInit);
};

void initTimer(){
	USTIMER_Init();
	// Initialization of RTCDRV driver
	RTCDRV_Init();

	// Reserve a timer
	RTCDRV_AllocateTimer( &id );
};

uint32_t millis(){
	return(RTCDRV_GetWallClockTicks32()/4);
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

void initIDAC(){
	CMU_ClockEnable(cmuClock_IDAC0, true);

	IDAC_Init(IDAC0, &DACInit);

	// Choose the output current to be 2 microamps
	IDAC_RangeSet(IDAC0, idacCurrentRange1);
	IDAC_StepSet(IDAC0, 4);
	IDAC_Enable(IDAC0, true);
}

void setIDAC(unsigned int pin_no, uint8_t step){
	DACInit.outMode = port_pin[pin_no].dac; // Choose output to be on PB8
	// Choose the output current to be 2 microamps
	IDAC_RangeSet(IDAC0, idacCurrentRange1);
	IDAC_StepSet(IDAC0, step);

	IDAC_Init(IDAC0, &DACInit);

	// Enable IDAC output mode and also enable the IDAC module itself
	IDAC_OutEnable(IDAC0, true);
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
	return GPIO_PinInGet(port_pin[pin_no].port, port_pin[pin_no].pin);
};

void digitalWrite(unsigned int pin_no, unsigned int value){
	GPIO_PinModeSet(port_pin[pin_no].port, port_pin[pin_no].pin , gpioModePushPull, value);
};

/* Analog I/O */

void analogWrite(unsigned int pin_no, byte value){
	if(port_pin[pin_no].dac != * IDACNone){
		if(port_pin[pin_no].state == MODE_ANALOG_OUT){
			// setup IDAC
		}
		else {
			port_pin[pin_no].state = MODE_ANALOG_OUT;

		};
	}
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



