#include <platform/emlib/inc/em_cmu.h>
#include <platform/emlib/inc/em_emu.h>
//#include "init_mcu.h"
//#include "init_board.h"
//#include "init_app.h"
//#include "board_features.h"
//#include "hal-config.h"
#include "uartdrv.h"

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

//typedef uint16_t size_t;

#define EMDRV_UARTDRV_MAX_CONCURRENT_BUFS 6
//DEFINE_BUF_QUEUE(EMDRV_UARTDRV_MAX_CONCURRENT_RX_BUFS, rxBufferQueue);
//DEFINE_BUF_QUEUE(EMDRV_UARTDRV_MAX_CONCURRENT_TX_BUFS, txBufferQueue);

namespace serial {

class SerialClass{

	UARTDRV_HandleData_t uarthandleData;
	UARTDRV_Handle_t uartHandle = &uarthandleData;

private:
	struct BufferQueueStruct{
		uint16_t head;
		uint16_t tail;
		volatile uint16_t used;
		const uint16_t size;
		UARTDRV_Buffer_t fifo[EMDRV_UARTDRV_MAX_CONCURRENT_BUFS];
	};

	static volatile BufferQueueStruct rxBufferQueue;
	static volatile BufferQueueStruct txBufferQueue;
public:
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


	size_t write(uint8_t * c);
	inline size_t write(unsigned long n);
	inline size_t write(long n);
	inline size_t write(unsigned int n);
	inline size_t write(int n);
	size_t write(const uint8_t *buffer, size_t size);
	size_t write(const char *buffer);

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

};

#ifdef __cplusplus
}
#endif
