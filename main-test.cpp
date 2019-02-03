#include "balena.h"
#include "Firmata.h"
#include "Serial.h"


SerialClass Serial;
//FirmataClass Firmata;
/* Declare some strings */
const char     welcomeString[]  = "balenaFin Firmata Test...\r\n";
const uint32_t welLen           = sizeof(welcomeString) - 1;

/* Declare a circular buffer structure to use for Rx and Tx queues */
#define BUFFERSIZE          256
#define I2C_WRITE                   B00000000
#define I2C_READ                    B00001000
#define I2C_READ_CONTINUOUSLY       B00010000
#define I2C_STOP_READING            B00011000
#define I2C_READ_WRITE_MODE_MASK    B00011000
#define I2C_10BIT_ADDRESS_MODE_MASK B00100000
#define I2C_END_TX_MASK             B01000000
#define I2C_STOP_TX                 1
#define I2C_RESTART_TX              0
#define I2C_MAX_QUERIES             8
#define I2C_REGISTER_NOT_SPECIFIED  -1

// the minimum interval for sampling analog input
#define MINIMUM_SAMPLING_INTERVAL 1

/******************************************************************************
 * @brief  Global Variables
 *
 *****************************************************************************/
#ifdef FIRMATA_SERIAL_FEATURE
SerialFirmata serialFeature;
#endif

/* analog inputs */
int analogInputsToReport = 0; // bitwise array to store pin reporting

/* digital input ports */
byte reportPINs[TOTAL_PORTS];       // 1 = report this port, 0 = silence
byte previousPINs[TOTAL_PORTS];     // previous 8 bits sent

/* pins configuration */
byte portConfigInputs[TOTAL_PORTS]; // each bit: 1 = pin in INPUT, 0 = anything else

/* timer variables */
unsigned long currentMillis;        // store the current value from millis()
unsigned long previousMillis;       // for comparison with currentMillis
unsigned int samplingInterval = 19; // how often to run the main loop (in ms)

/* i2c data */
struct i2c_device_info {
  byte addr;
  int reg;
  byte bytes;
  byte stopTX;
};

/* for i2c read continuous more */
i2c_device_info query[I2C_MAX_QUERIES];

byte i2cRxData[64];
bool isI2CEnabled = false;
signed char queryIndex = -1;
// default delay time between i2c read request and Wire.requestFrom()
unsigned int i2cReadDelayTime = 0;

bool isResetting = false;

// Forward declare a few functions to avoid compiler errors with older versions
// of the Arduino IDE.
void setPinModeCallback(byte, int);
void reportAnalogCallback(byte analogPin, int value);
void sysexCallback(byte, byte, byte*);

/******************************************************************************
 * @brief  Firmata Functions
 *
 *****************************************************************************/

void outputPort(byte portNumber, byte portValue, byte forceSend)
{
  // pins not configured as INPUT are cleared to zeros
  portValue = portValue & portConfigInputs[portNumber];
  // only send if the value is different than previously sent
  if (forceSend || previousPINs[portNumber] != portValue) {
    Firmata.sendDigitalPort(portNumber, portValue);
    previousPINs[portNumber] = portValue;
  }
}

void setPinModeCallback(byte pin, int mode)
{
  if (Firmata.getPinMode(pin) == PIN_MODE_IGNORE)
    return;

//  if (Firmata.getPinMode(pin) == PIN_MODE_I2C && isI2CEnabled && mode != PIN_MODE_I2C) {
//    // disable i2c so pins can be used for other functions
//    // the following if statements should reconfigure the pins properly
//    disableI2CPins();
//  }
//  if (IS_PIN_DIGITAL(pin) && mode != PIN_MODE_SERVO) {
//    if (servoPinMap[pin] < MAX_SERVOS && servos[servoPinMap[pin]].attached()) {
//      detachServo(pin);
//    }
//  }
  if (IS_PIN_ANALOG(pin)) {
    reportAnalogCallback(PIN_TO_ANALOG(pin), mode == PIN_MODE_ANALOG ? 1 : 0); // turn on/off reporting
  }
  if (IS_PIN_DIGITAL(pin)) {
    if (mode == INPUT || mode == PIN_MODE_PULLUP) {
      portConfigInputs[pin / 8] |= (1 << (pin & 7));
    } else {
      portConfigInputs[pin / 8] &= ~(1 << (pin & 7));
    }
  }
  Firmata.setPinState(pin, 0);
  switch (mode) {
    case PIN_MODE_ANALOG:
      if (IS_PIN_ANALOG(pin)) {
        if (IS_PIN_DIGITAL(pin)) {
          pinMode(PIN_TO_DIGITAL(pin), INPUT);    // disable output driver
#if ARDUINO <= 100
          // deprecated since Arduino 1.0.1 - TODO: drop support in Firmata 2.6
          digitalWrite(PIN_TO_DIGITAL(pin), LOW); // disable internal pull-ups
#endif
        }
        Firmata.setPinMode(pin, PIN_MODE_ANALOG);
      }
      break;
    case INPUT:
      if (IS_PIN_DIGITAL(pin)) {
        pinMode(PIN_TO_DIGITAL(pin), INPUT);    // disable output driver
#if ARDUINO <= 100
        // deprecated since Arduino 1.0.1 - TODO: drop support in Firmata 2.6
        digitalWrite(PIN_TO_DIGITAL(pin), LOW); // disable internal pull-ups
#endif
        Firmata.setPinMode(pin, INPUT);
      }
      break;
    case PIN_MODE_PULLUP:
      if (IS_PIN_DIGITAL(pin)) {
        pinMode(PIN_TO_DIGITAL(pin), INPUT_PULLUP);
        Firmata.setPinMode(pin, PIN_MODE_PULLUP);
        Firmata.setPinState(pin, 1);
      }
      break;
    case OUTPUT:
      if (IS_PIN_DIGITAL(pin)) {
        if (Firmata.getPinMode(pin) == PIN_MODE_PWM) {
          // Disable PWM if pin mode was previously set to PWM.
          digitalWrite(PIN_TO_DIGITAL(pin), LOW);
        }
        pinMode(PIN_TO_DIGITAL(pin), OUTPUT);
        Firmata.setPinMode(pin, OUTPUT);
      }
      break;
    case PIN_MODE_PWM:
      if (IS_PIN_PWM(pin)) {
        pinMode(PIN_TO_PWM(pin), OUTPUT);
        analogWrite(PIN_TO_PWM(pin), 0);
        Firmata.setPinMode(pin, PIN_MODE_PWM);
      }
      break;
//    case PIN_MODE_SERVO:
//      if (IS_PIN_DIGITAL(pin)) {
//        Firmata.setPinMode(pin, PIN_MODE_SERVO);
//        if (servoPinMap[pin] == 255 || !servos[servoPinMap[pin]].attached()) {
//          // pass -1 for min and max pulse values to use default values set
//          // by Servo library
//          attachServo(pin, -1, -1);
//        }
//      }
//      break;
//    case PIN_MODE_I2C:
//      if (IS_PIN_I2C(pin)) {
//        // mark the pin as i2c
//        // the user must call I2C_CONFIG to enable I2C for a device
//        Firmata.setPinMode(pin, PIN_MODE_I2C);
//      }
//      break;
    case PIN_MODE_SERIAL:
#ifdef FIRMATA_SERIAL_FEATURE
      serialFeature.handlePinMode(pin, PIN_MODE_SERIAL);
#endif
      break;
    default:
      Firmata.sendString("Unknown pin mode"); // TODO: put error msgs in EEPROM
  }
  // TODO: save status to EEPROM here, if changed
}

void setPinValueCallback(byte pin, int value)
{
  if (pin < TOTAL_PINS && IS_PIN_DIGITAL(pin)) {
    if (Firmata.getPinMode(pin) == OUTPUT) {
      Firmata.setPinState(pin, value);
      digitalWrite(PIN_TO_DIGITAL(pin), value);
    }
  }
}

void reportAnalogCallback(byte analogPin, int value)
{
	digitalWrite(3,0);
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
  // TODO: save status to EEPROM here, if changed
}

void reportDigitalCallback(byte port, int value)
{
  if (port < TOTAL_PORTS) {
    reportPINs[port] = (byte)value;
    // Send port value immediately. This is helpful when connected via
    // ethernet, wi-fi or bluetooth so pin states can be known upon
    // reconnecting.
    if (value) outputPort(port, readPort(port, portConfigInputs[port]), true);
  }
  // do not disable analog reporting on these 8 pins, to allow some
  // pins used for digital, others analog.  Instead, allow both types
  // of reporting to be enabled, but check if the pin is configured
  // as analog when sampling the analog inputs.  Likewise, while
  // scanning digital pins, portConfigInputs will mask off values from any
  // pins configured as analog
}

/******************************************************************************
 * @brief  Sysex Commands
 *
 *****************************************************************************/

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
          Firmata.write((byte)INPUT);
          Firmata.write(1);
          Firmata.write((byte)PIN_MODE_PULLUP);
          Firmata.write(1);
          Firmata.write((byte)OUTPUT);
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
          Firmata.write(1);
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

void systemResetCallback()
{
  isResetting = true;

  // initialize a defalt state
  // TODO: option to load config from EEPROM instead of default

#ifdef FIRMATA_SERIAL_FEATURE
  serialFeature.reset();
#endif

//  if (isI2CEnabled) {
//    disableI2CPins();
//  }

  for (byte i = 0; i < TOTAL_PORTS; i++) {
    reportPINs[i] = false;    // by default, reporting off
    portConfigInputs[i] = 0;  // until activated
    previousPINs[i] = 0;
  }

  for (byte i = 0; i < TOTAL_PINS; i++) {
    // pins with analog capability default to analog input
    // otherwise, pins default to digital output
    if (IS_PIN_ANALOG(i)) {
      // turns off pullup, configures everything
      setPinModeCallback(i, PIN_MODE_ANALOG);
    } else if (IS_PIN_DIGITAL(i)) {
      // sets the output to 0, configures portConfigInputs
      setPinModeCallback(i, OUTPUT);
    }

//    servoPinMap[i] = 255;
  }
  // by default, do not report any analog inputs
  analogInputsToReport = 0;

//  detachedServoCount = 0;
//  servoCount = 0;

  /* send digital inputs to set the initial state on the host computer,
   * since once in the loop(), this firmware will only send on change */
  /*
  TODO: this can never execute, since no pins default to digital input
        but it will be needed when/if we support EEPROM stored config
  for (byte i=0; i < TOTAL_PORTS; i++) {
    outputPort(i, readPort(i, portConfigInputs[i]), true);
  }
  */
  isResetting = false;
}

/******************************************************************************
 * @brief  Main function
 *
 *****************************************************************************/
int main(void)
{
	Firmata.setFirmwareVersion(FIRMATA_FIRMWARE_MAJOR_VERSION, FIRMATA_FIRMWARE_MINOR_VERSION);
//	Firmata.attach(ANALOG_MESSAGE, analogWriteCallback);
//	Firmata.attach(DIGITAL_MESSAGE, digitalWriteCallback);
	Firmata.attach(REPORT_ANALOG, reportAnalogCallback);
	Firmata.attach(REPORT_DIGITAL, reportDigitalCallback);
	Firmata.attach(SET_PIN_MODE, setPinModeCallback);
//	Firmata.attach(SET_DIGITAL_PIN_VALUE, setPinValueCallback);
	Firmata.attach(START_SYSEX, sysexCallback);
	Firmata.attach(SYSTEM_RESET, systemResetCallback);

	balenaInit();

	digitalWrite(3,1);

	Serial.begin(57600);;

	Firmata.begin(Serial);

	while(1){
		  if(Firmata.available() > 0){
		     Firmata.processInput();
		  }
		  else{

		  }
//		  Serial.write(i);
////		  Serial.write(analogRead(5));
//		  Serial.write(nl,2);
//		  delay(1000);
	  }

//  while (1)
//  {
////    /* Wait in EM1 while UART transmits */
//////    EMU_EnterEM1();
////    /* Check if termination character is received */
//    if (Serial.available() > 0)
//    {
//      /* Copy received data to UART transmit queue */
//      uint8_t tmpBuf[BUFFERSIZE];
//      unsigned long len = Serial.readBytes(tmpBuf, 0);
//      Serial.write(tmpBuf, len);
//    }
//  }
}


