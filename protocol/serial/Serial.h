#ifndef __SERIAL_H__
#define __SERIAL_H__

#include "hal-config.h"
#include "em_gpio.h"
#include "em_usart.h"
#include "em_chip.h"
#include "em_cmu.h"
#include "em_device.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

#if defined(__cplusplus) && !defined(ARDUINO)
  #include <cstddef>
  #include <cstdint>
#else
  #include <stddef.h>
  #include <stdint.h>
#endif

#define VCOM 1
#define CM3 0

#define EMDRV_UARTDRV_MAX_CONCURRENT_BUFS 12
#define BUFFER_SIZE 80

extern volatile uint32_t rx_data_ready;
extern volatile uint32_t rx_data_available;
extern volatile uint32_t tx_data_available;

extern volatile char rx_buffer[BUFFER_SIZE];
extern volatile char tx_buffer[BUFFER_SIZE];

typedef unsigned char byte;


class SerialClass{
private:
USART_InitAsync_TypeDef uartInit =
{                                                                                                  \
	usartDisable,           /* Enable RX/TX when initialization is complete. */                    \
	0,                     /* Use current configured reference clock for configuring baud rate. */ \
	115200,                /* 115200 bits/s. */                                                    \
	usartOVS16,            /* 16x oversampling. */                                                 \
	usartDatabits8,        /* 8 data bits. */                                                      \
	usartNoParity,         /* No parity. */                                                        \
	usartStopbits1,        /* 1 stop bit. */                                                       \
	false,                 /* Do not disable majority vote. */                                     \
	false,                 /* Not USART PRS input mode. */                                         \
	0,                     /* PRS channel 0. */                                                    \
	false,                 /* Auto CS functionality enable/disable switch */                       \
	0,                     /* Auto CS Hold cycles */                                               \
	0,                     /* Auto CS Setup cycles */                                              \
	usartHwFlowControlNone /* No HW flow control */                                                \
 };

public:
    SerialClass();
    SerialClass(unsigned int interface, long baudrate);
	int available(void);
	int availableForWrite(void);
	void begin(long baudrate);
	void end();

	int peek(void);
	void flush(void);

	int baudRate(void);
	size_t getRxBufferSize();

	int read(void);
	uint32_t readBytes(uint8_t * dataPtr, uint32_t dataLen);
	size_t readBytesUntil();
	size_t readString();
	size_t readStringUntil();


	void write(uint8_t ch);
	void write(uint8_t * dataPtr, uint32_t dataLen);

	void setDebugOutput(bool);

	bool isTxEnabled(void);
	bool isRxEnabled(void);
	bool hasOverrun(void);
	bool hasRxError(void);

	void find();
	void findUntil();

	void setTimeout();
	size_t write();
	void serialEvent();

	void parseFloat();
	void parseInt();

};

#ifdef __cplusplus
}
#endif
#endif
