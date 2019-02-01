#include "balena.h"
#include "Firmata.h"
//#include "Serial.h"

/*
 * Firmata is a generic protocol for communicating with microcontrollers
 * from software on a host computer. It is intended to work with
 * any host computer software package.
 *
 * To download a host software package, please click on the following link
 * to open the list of Firmata client libraries in your default browser.
 *
 * https://github.com/firmata/arduino#firmata-client-libraries
 */

/* Supports as many digital inputs and outputs as possible.
 *
 * This example code is in the public domain.
 */

#define I2C_WRITE                   0x00
#define I2C_READ                    0x08
#define I2C_READ_CONTINUOUSLY       0x10
#define I2C_STOP_READING            0x18
#define I2C_READ_WRITE_MODE_MASK    0x18
#define I2C_10BIT_ADDRESS_MODE_MASK 0x20
#define I2C_END_TX_MASK             0x40
#define I2C_STOP_TX                 1
#define I2C_RESTART_TX              0
#define I2C_MAX_QUERIES             8
#define I2C_REGISTER_NOT_SPECIFIED  -1

// the minimum interval for sampling analog input
#define MINIMUM_SAMPLING_INTERVAL 1

SerialClass Serial(1);
firmata::FirmataClass Firmata;

byte previousPIN[TOTAL_PORTS];  // PIN means PORT for input
byte previousPORT[TOTAL_PORTS];

int analogInputsToReport = 0; // bitwise array to store pin reporting
bool isResetting = false;

void outputPort(byte portNumber, byte portValue)
{
  // only send the data when it changes, otherwise you get too many messages!
  if (previousPIN[portNumber] != portValue) {
    Firmata.sendDigitalPort(portNumber, portValue);
    previousPIN[portNumber] = portValue;
  }
}

void setPinModeCallback(byte pin, int mode) {
  if (IS_PIN_DIGITAL(pin)) {
    pinMode(PIN_TO_DIGITAL(pin), mode);
  }
}

void digitalWriteCallback(byte port, int value)
{
  byte i;
  byte currentPinValue, previousPinValue;

  if (port < TOTAL_PORTS && value != previousPORT[port]) {
    for (i = 0; i < 8; i++) {
      currentPinValue = (byte) value & (1 << i);
      previousPinValue = previousPORT[port] & (1 << i);
      if (currentPinValue != previousPinValue) {
        digitalWrite(i + (port * 8), currentPinValue);
      }
    }
    previousPORT[port] = value;
  }
}

void reportAnalogCallback(byte analogPin, int value)
{
  if (analogPin < TOTAL_ANALOG_PINS) {
    if (value == 0) {
      analogInputsToReport = analogInputsToReport & ~ (1 << analogPin);
    } else {
      analogInputsToReport = analogInputsToReport | (1 << analogPin);
      // prevent during system reset or all analog pin values will be reported
      // which may report noise for unconnected analog pins
      if (!isResetting) {
        // Send pin value immediately. This is helpful when connected via
        // ethernet, wi-fi or bluetooth so pin states can be known upon
        // reconnecting.
        Firmata.sendAnalog(analogPin, analogRead(analogPin));
      }
    }
  }
}

/* Add sysex callback implementation */

void sysexCallback(byte command, byte argc, byte *argv)
{
  byte mode;
  byte stopTX;
  byte slaveAddress;
  byte data;
  int slaveRegister;
  unsigned int delayTime;

  switch (command) {
//    case I2C_REQUEST:
//      mode = argv[1] & I2C_READ_WRITE_MODE_MASK;
//      if (argv[1] & I2C_10BIT_ADDRESS_MODE_MASK) {
//        Firmata.sendString("10-bit addressing not supported");
//        return;
//      }
//      else {
//        slaveAddress = argv[0];
//      }
//
//      // need to invert the logic here since 0 will be default for client
//      // libraries that have not updated to add support for restart tx
//      if (argv[1] & I2C_END_TX_MASK) {
//        stopTX = I2C_RESTART_TX;
//      }
//      else {
//        stopTX = I2C_STOP_TX; // default
//      }
//
//      switch (mode) {
//        case I2C_WRITE:
//          Wire.beginTransmission(slaveAddress);
//          for (byte i = 2; i < argc; i += 2) {
//            data = argv[i] + (argv[i + 1] << 7);
//            wireWrite(data);
//          }
//          Wire.endTransmission();
//          delayMicroseconds(70);
//          break;
//        case I2C_READ:
//          if (argc == 6) {
//            // a slave register is specified
//            slaveRegister = argv[2] + (argv[3] << 7);
//            data = argv[4] + (argv[5] << 7);  // bytes to read
//          }
//          else {
//            // a slave register is NOT specified
//            slaveRegister = I2C_REGISTER_NOT_SPECIFIED;
//            data = argv[2] + (argv[3] << 7);  // bytes to read
//          }
//          readAndReportData(slaveAddress, (int)slaveRegister, data, stopTX);
//          break;
//        case I2C_READ_CONTINUOUSLY:
//          if ((queryIndex + 1) >= I2C_MAX_QUERIES) {
//            // too many queries, just ignore
//            Firmata.sendString("too many queries");
//            break;
//          }
//          if (argc == 6) {
//            // a slave register is specified
//            slaveRegister = argv[2] + (argv[3] << 7);
//            data = argv[4] + (argv[5] << 7);  // bytes to read
//          }
//          else {
//            // a slave register is NOT specified
//            slaveRegister = (int)I2C_REGISTER_NOT_SPECIFIED;
//            data = argv[2] + (argv[3] << 7);  // bytes to read
//          }
//          queryIndex++;
//          query[queryIndex].addr = slaveAddress;
//          query[queryIndex].reg = slaveRegister;
//          query[queryIndex].bytes = data;
//          query[queryIndex].stopTX = stopTX;
//          break;
//        case I2C_STOP_READING:
//          byte queryIndexToSkip;
//          // if read continuous mode is enabled for only 1 i2c device, disable
//          // read continuous reporting for that device
//          if (queryIndex <= 0) {
//            queryIndex = -1;
//          } else {
//            queryIndexToSkip = 0;
//            // if read continuous mode is enabled for multiple devices,
//            // determine which device to stop reading and remove it's data from
//            // the array, shifiting other array data to fill the space
//            for (byte i = 0; i < queryIndex + 1; i++) {
//              if (query[i].addr == slaveAddress) {
//                queryIndexToSkip = i;
//                break;
//              }
//            }
//
//            for (byte i = queryIndexToSkip; i < queryIndex + 1; i++) {
//              if (i < I2C_MAX_QUERIES) {
//                query[i].addr = query[i + 1].addr;
//                query[i].reg = query[i + 1].reg;
//                query[i].bytes = query[i + 1].bytes;
//                query[i].stopTX = query[i + 1].stopTX;
//              }
//            }
//            queryIndex--;
//          }
//          break;
//        default:
//          break;
//      }
//      break;
//    case I2C_CONFIG:
//      delayTime = (argv[0] + (argv[1] << 7));
//
//      if (argc > 1 && delayTime > 0) {
//        i2cReadDelayTime = delayTime;
//      }
//
//      if (!isI2CEnabled) {
//        enableI2CPins();
//      }
//
//      break;
//    case SERVO_CONFIG:
//      if (argc > 4) {
//        // these vars are here for clarity, they'll optimized away by the compiler
//        byte pin = argv[0];
//        int minPulse = argv[1] + (argv[2] << 7);
//        int maxPulse = argv[3] + (argv[4] << 7);
//
//        if (IS_PIN_DIGITAL(pin)) {
//          if (servoPinMap[pin] < MAX_SERVOS && servos[servoPinMap[pin]].attached()) {
//            detachServo(pin);
//          }
//          attachServo(pin, minPulse, maxPulse);
//          setPinModeCallback(pin, PIN_MODE_SERVO);
//        }
//      }
//      break;
//    case SAMPLING_INTERVAL:
//      if (argc > 1) {
//        samplingInterval = argv[0] + (argv[1] << 7);
//        if (samplingInterval < MINIMUM_SAMPLING_INTERVAL) {
//          samplingInterval = MINIMUM_SAMPLING_INTERVAL;
//        }
//      } else {
//        //Firmata.sendString("Not enough data");
//      }
//      break;
//    case EXTENDED_ANALOG:
//      if (argc > 1) {
//        int val = argv[1];
//        if (argc > 2) val |= (argv[2] << 7);
//        if (argc > 3) val |= (argv[3] << 14);
//        analogWriteCallback(argv[0], val);
//      }
//      break;
    case CAPABILITY_QUERY:
      Firmata.write(START_SYSEX);
      Firmata.write(CAPABILITY_RESPONSE);
      for (byte pin = 0; pin < TOTAL_PINS; pin++) {
        if (IS_PIN_DIGITAL(pin)) {
          Firmata.write((byte)PIN_MODE_INPUT);
          Firmata.write(1);
          Firmata.write((byte)PIN_MODE_PULLUP);
          Firmata.write(1);
          Firmata.write((byte)PIN_MODE_OUTPUT);
          Firmata.write(1);
        }
        if (IS_PIN_ANALOG(pin)) {
          Firmata.write(PIN_MODE_ANALOG);
          Firmata.write(10); // 10 = 10-bit resolution
        }
        if (IS_PIN_PWM(pin)) {
          Firmata.write(PIN_MODE_PWM);
          Firmata.write(DEFAULT_PWM_RESOLUTION);
        }
        if (IS_PIN_DIGITAL(pin)) {
          Firmata.write(PIN_MODE_SERVO);
          Firmata.write(14);
        }
        if (IS_PIN_I2C(pin)) {
          Firmata.write(PIN_MODE_I2C);
          Firmata.write(1);  // TODO: could assign a number to map to SCL or SDA
        }
#ifdef FIRMATA_SERIAL_FEATURE
        serialFeature.handleCapability(pin);
#endif
        Firmata.write(127);
      }
      Firmata.write(END_SYSEX);
      break;
    case PIN_STATE_QUERY:
      if (argc > 0) {
        byte pin = argv[0];
        Firmata.write(START_SYSEX);
        Firmata.write(PIN_STATE_RESPONSE);
        Firmata.write(pin);
        if (pin < TOTAL_PINS) {
          Firmata.write(Firmata.getPinMode(pin));
          Firmata.write((byte)Firmata.getPinState(pin) & 0x7F);
          if (Firmata.getPinState(pin) & 0xFF80) Firmata.write((byte)(Firmata.getPinState(pin) >> 7) & 0x7F);
          if (Firmata.getPinState(pin) & 0xC000) Firmata.write((byte)(Firmata.getPinState(pin) >> 14) & 0x7F);
        }
        Firmata.write(END_SYSEX);
      }
      break;
    case ANALOG_MAPPING_QUERY:
      Firmata.write(START_SYSEX);
      Firmata.write(ANALOG_MAPPING_RESPONSE);
      for (byte pin = 0; pin < TOTAL_PINS; pin++) {
        Firmata.write(IS_PIN_ANALOG(pin) ? PIN_TO_ANALOG(pin) : 127);
      }
      Firmata.write(END_SYSEX);
      break;

    case SERIAL_MESSAGE:
#ifdef FIRMATA_SERIAL_FEATURE
      serialFeature.handleSysex(command, argc, argv);
#endif
      break;
  }
}

int main_test(void)
{
	long baud = 115200;
	int i = 0;
		  unsigned char welcome_string[] = "Silicon Labs UART Code test!\r\f";
	Firmata.setFirmwareVersion(FIRMATA_FIRMWARE_MAJOR_VERSION, FIRMATA_FIRMWARE_MINOR_VERSION);
	Firmata.attach(DIGITAL_MESSAGE, digitalWriteCallback);
	Firmata.attach(START_SYSEX, sysexCallback);
	Firmata.attach(SET_PIN_MODE, setPinModeCallback);
//////	Firmata.begin(baud);

	Serial.begin(baud);
		Firmata.begin(Serial);

//	  // Print welcome message
//	  for (i = 0 ; welcome_string[i] != 0; i++)
//	  {
//	    tx_buffer[i] = welcome_string[i];
//	  }
//	  USART_IntSet(USART0, USART_IFS_TXC);
//
//	  while (1)
//	  {
//	    /* When notified by the RX handler, copy data from the RX buffer to the
//	     * TX buffer, and start the TX handler */
//	    if (1)
//	    {
//
//	      USART_IntDisable(USART0, USART_IEN_RXDATAV);
//	      USART_IntDisable(USART0, USART_IEN_TXC);
//			digitalWrite(3,0);
//
//	      for (i = 0; rx_buffer[i] != 0 && i < BUFFER_SIZE-3; i++)
//	      {
//	        tx_buffer[i] = rx_buffer[i];
//	      }
//	      tx_buffer[i++] = (char)223;
//	      tx_buffer[i++] = '\r';
//	      tx_buffer[i++] = '\f';
//	      tx_buffer[i] = '\0';
//	      rx_data_ready = 0;
//
//	      USART_IntEnable(USART0, USART_IEN_RXDATAV);
//	      USART_IntEnable(USART0, USART_IEN_TXC);
//	      USART_IntSet(USART0, USART_IFS_TXC);
//	    }
//	}
//
//	while(1){
////		Serial.readBytes(data, 12);
////		Serial.write(data);
//	}
//	Firmata.begin(Serial);




	  while (1) {

		  byte i;

//		   for (i = 0; i < TOTAL_PORTS; i++) {
//		     outputPort(i, readPort(i, 0xff));
//		   }

		   while (Firmata.available() != 0) {
				digitalWrite(3,0);
		     Firmata.processInput();
		   }
			digitalWrite(3,1);

	  };

//	 USART_InitAsync_TypeDef init = USART_INITASYNC_DEFAULT;
//	  unsigned char welcome_string[] = "Silicon Labs UoART Code example!\r\f";
//	  uint32_t i;
//	  long baud = 115200;
//	  	Serial.begin(baud);
//
//	  	Serial.write(welcome_string);


	  // Chip errata
//	  CHIP_Init();

//	  // Enable oscillator to GPIO and USART0 modules
//	  CMU_ClockEnable(cmuClock_GPIO, true);
//	  CMU_ClockEnable(cmuClock_USART0, true);
//
//	  // set pin modes for USART TX and RX pins
//	  GPIO_PinModeSet(gpioPortA, 1, gpioModeInput, 0);
//	  GPIO_PinModeSet(gpioPortA, 0, gpioModePushPull, 1);
//
//	  // Initialize USART asynchronous mode and route pins
//	  USART_InitAsync(USART0, &init);
//	  USART0->ROUTEPEN |= USART_ROUTEPEN_TXPEN | USART_ROUTEPEN_RXPEN;
//	  USART0->ROUTELOC0 = USART_ROUTELOC0_RXLOC_LOC0 | USART_ROUTELOC0_TXLOC_LOC0;
//
//	  //Initializae USART Interrupts
//	  USART_IntEnable(USART0, USART_IEN_RXDATAV);
//	  USART_IntEnable(USART0, USART_IEN_TXC);
//
//	  //Enabling USART Interrupts
//	  NVIC_EnableIRQ(USART0_RX_IRQn);
//	  NVIC_EnableIRQ(USART0_TX_IRQn);

	  // Print welcome message
//	  for (i = 0 ; welcome_string[i] != 0; i++)
//	  {
//	    tx_buffer[i] = welcome_string[i];
//	  }
//	  USART_IntSet(USART0, USART_IFS_TXC);

  return 0;
}
