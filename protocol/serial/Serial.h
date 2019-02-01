//#ifndef __SERIAL_H__
//#define __SERIAL_H__
//
//#include "uartdrv.h"
//#include "uartdrv_config.h"
//#include "hal-config.h"
//#include "em_gpio.h"
//#include "em_usart.h"
//#include "em_chip.h"
//#include "em_cmu.h"
//#include "em_device.h"
//#include <string.h>
//#include <stdio.h>
//#include <stdlib.h>
//#include "balena.h"
//
//#ifdef __cplusplus
//extern "C" {
//#endif
//
//#if defined(__cplusplus) && !defined(ARDUINO)
//  #include <cstddef>
//  #include <cstdint>
//#else
//  #include <stddef.h>
//  #include <stdint.h>
//#endif
//
//#define VCOM 1
//#define CM3 0
//
//#define EMDRV_UARTDRV_MAX_CONCURRENT_BUFS 12
//#define BUFFER_SIZE 80
//
//#define CM3_CONFIG                                  \
//{                                                   \
//  USART0,                                           \
//  115200,                                           \
//  _USART_ROUTELOC0_TXLOC_LOC0,                      \
//  _USART_ROUTELOC0_RXLOC_LOC0,                      \
//  usartStopbits1,                                   \
//  usartNoParity,                                    \
//  usartOVS16,                                       \
//  false,                                            \
//  uartdrvFlowControlNone,							\
//  gpioPortA,                                        \
//  4,                                                \
//  gpioPortA,                                        \
//  5,                                                \
//  (UARTDRV_Buffer_FifoQueue_t *)&rxBufferQueue,     \
//  (UARTDRV_Buffer_FifoQueue_t *)&txBufferQueue,     \
//}
//
//#define VCOM_CONFIG                                \
//{                                                   \
//  USART0,                                           \
//  115200,                                           \
//  _USART_ROUTELOC0_TXLOC_LOC0,                      \
//  _USART_ROUTELOC0_RXLOC_LOC0,                      \
//  usartStopbits1,                                   \
//  usartNoParity,                                    \
//  usartOVS16,                                       \
//  false,                                            \
//  uartdrvFlowControlNone,							\
//  gpioPortA,                                        \
//  3,                                                \
//  gpioPortA,                                        \
//  5,                                                \
//  (UARTDRV_Buffer_FifoQueue_t *) &rxBufferQueue,   \
//  (UARTDRV_Buffer_FifoQueue_t *) &txBufferQueue    \
//}
//
//typedef struct {
//    uint16_t head;
//    uint16_t tail;
//    volatile uint16_t used;
//    const uint16_t size;
//    UARTDRV_Buffer_t fifo[12];
//  } _rxBufferQueueI0;
//
//typedef struct {
//  uint16_t head;
//  uint16_t tail;
//  volatile uint16_t used;
//  const uint16_t size;
//  UARTDRV_Buffer_t fifo[12];
//} _txBufferQueueI0;
//
//typedef unsigned char byte;
//
//class SerialClass{
//private:
//	UARTDRV_HandleData_t uarthandleData;
//	UARTDRV_Handle_t uartHandle = &uarthandleData;
//	UARTDRV_Init_t uartInitData;
//
//	_rxBufferQueueI0  rxBufferQueue = {0,0,12,12};
//	_txBufferQueueI0  txBufferQueue = {0,0,12,12};
//
//	volatile uint32_t rx_data_ready = 0;
//	unsigned char rx_buffer[BUFFER_SIZE];
//	volatile char tx_buffer[BUFFER_SIZE];
//
//	UARTDRV_Count_t RXReceived = 0;
//	UARTDRV_Count_t RXRemaining = 0;
//
//public:
//    SerialClass (unsigned int interface);
//    SerialClass (unsigned int interface, long baudrate);
//	int available(void);
//	int availableForWrite(void);
//	void begin(long baudrate);
//	void end();
//	void USART0_RX_IRQHandler(void);
//
//	int peek(void);
//	void flush(void);
//
//	int baudRate(void);
//	size_t getRxBufferSize();
//
//	int read(void);
////	size_t readBytes(char* buffer, size_t size);
//	size_t readBytes(uint8_t* buffer, size_t size);
//	size_t readBytesUntil();
//	size_t readString();
//	size_t readStringUntil();
//
//
//	size_t write(unsigned char * c);
////	inline size_t write(unsigned long n);
////	inline size_t write(long n);
////	inline size_t write(unsigned int n);
//	size_t write(int n);
//	size_t write(int * n);
////	size_t write(const uint8_t *buffer, size_t size);
////	size_t write(const char *buffer);
//
//	void setDebugOutput(bool);
//
//	bool isTxEnabled(void);
//	bool isRxEnabled(void);
//	bool hasOverrun(void);
//	bool hasRxError(void);
//
//	void find();
//	void findUntil();
//
//	void setTimeout();
//	size_t write();
//	void serialEvent();
//
//	void parseFloat();
//	void parseInt();
//
//};
//
//#ifdef __cplusplus
//}
//#endif
//#endif

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
//#include "balena.h"

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

//extern void USART0_RX_IRQHandler(void);
//extern void USART0_TX_IRQHandler(void);

// https://github.com/hrshygoodness/EFM32-Library/blob/master/v2/an/an0013_efm32_dma/uart_tx_rx.c

typedef unsigned char byte;


class SerialClass{
private:
USART_InitAsync_TypeDef uartInit =
{                                                                                                \
	usartDisable,           /* Enable RX/TX when initialization is complete. */                     \
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

	uint8_t read(void);
//	size_t readBytes(char* buffer, size_t size);
	uint32_t readBytes(uint8_t * dataPtr, uint32_t dataLen);
	size_t readBytesUntil();
	size_t readString();
	size_t readStringUntil();


	void write(uint8_t ch);
	void write(uint8_t * dataPtr, uint32_t dataLen);

//	inline size_t write(unsigned long n);
//	inline size_t write(long n);
//	inline size_t write(unsigned int n);
//	size_t write(int n);
//	size_t write(int * n);
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
