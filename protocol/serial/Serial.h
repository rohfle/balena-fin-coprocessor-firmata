#ifndef SERIAL_H
#define SERIAL_H

#include "uartdrv.h"
#include "uartdrv_config.h"
#include "em_gpio.h"
#include "hal-config.h"
#include "em_chip.h"
#include "em_cmu.h"
#include "em_device.h"

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

#define EMDRV_UARTDRV_MAX_CONCURRENT_BUFS 6

#define CM3_CONFIG                                  \
{                                                   \
  USART0,                                           \
  baud,                                             \
  _USART_ROUTELOC0_TXLOC_LOC0,                      \
  _USART_ROUTELOC0_RXLOC_LOC0,                      \
  usartStopbits1,                                   \
  usartNoParity,                                    \
  usartOVS16,                                       \
  false,                                            \
  uartdrvFlowControlNone,							\
  gpioPortA,                                        \
  4,                                                \
  gpioPortA,                                        \
  5,                                                \
  (UARTDRV_Buffer_FifoQueue_t *)&rxBufferQueue,     \
  (UARTDRV_Buffer_FifoQueue_t *)&txBufferQueue,     \
}

#define VCOM_CONFIG                                \
{                                                   \
  USART0,                                           \
  115200,                                           \
  _USART_ROUTELOC0_TXLOC_LOC0,                      \
  _USART_ROUTELOC0_RXLOC_LOC0,                      \
  usartStopbits1,                                   \
  usartNoParity,                                    \
  usartOVS16,                                       \
  false,                                            \
  uartdrvFlowControlNone,							\
  gpioPortA,                                        \
  3,                                                \
  gpioPortA,                                        \
  5,                                                \
  (UARTDRV_Buffer_FifoQueue_t *) &rxBufferQueue,   \
  (UARTDRV_Buffer_FifoQueue_t *) &txBufferQueue    \
}

typedef struct {
    uint16_t head;
    uint16_t tail;
    volatile uint16_t used;
    const uint16_t size;
    UARTDRV_Buffer_t fifo[6];
  } _rxBufferQueueI0;

typedef struct {
  uint16_t head;
  uint16_t tail;
  volatile uint16_t used;
  const uint16_t size;
  UARTDRV_Buffer_t fifo[6];
} _txBufferQueueI0;

typedef unsigned char byte;


class SerialClass{
	UARTDRV_HandleData_t uarthandleData;
	UARTDRV_Handle_t uartHandle = &uarthandleData;
	UARTDRV_Init_t uartInitData;

private:
	_rxBufferQueueI0  rxBufferQueue = {0,0,0,6};
	_txBufferQueueI0  txBufferQueue = {0,0,0,6};

public:
    SerialClass (unsigned int);
    SerialClass (unsigned int, unsigned int);
	unsigned long baud;

	int available(void);
	int availableForWrite(void);
	void begin(unsigned long baud);
	void end();

	int peek(void);
	void flush(void);

	int baudRate(void);
	size_t getRxBufferSize();

	int read(void);
//	size_t readBytes(char* buffer, size_t size);
	size_t readBytes(uint8_t* buffer, size_t size);
	size_t readBytesUntil();
	size_t readString();
	size_t readStringUntil();


	size_t write(unsigned char * c);
//	inline size_t write(unsigned long n);
//	inline size_t write(long n);
//	inline size_t write(unsigned int n);
	size_t write(int n);
	inline size_t write(int * n);
//	size_t write(const uint8_t *buffer, size_t size);
//	size_t write(const char *buffer);

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
