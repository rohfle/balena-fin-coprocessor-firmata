#include "Serial.h"

//uint8_t buffer[64] = "test\r\n";

using namespace serial;

SerialClass Serial;

void SerialClass::begin(unsigned long baud){
    UARTDRV_Init_t uartInitData = CM3;
    UARTDRV_Init(uartHandle, &uartInitData);
};

size_t SerialClass::write(uint8_t * c){
	size_t status = UARTDRV_TransmitB(uartHandle, c, (sizeof(c)/sizeof(uint8_t)));
	return status;
};

size_t SerialClass::readBytes(uint8_t* buffer, size_t size){
	size_t status = UARTDRV_ReceiveB(uartHandle,buffer,size);
	return status;
};

int SerialClass::available(void){
	UARTDRV_Status_t count = UARTDRV_GetReceiveDepth(uartHandle);
	return count;
};

int SerialClass::availableForWrite(void){
	return (txBufferQueue.size - txBufferQueue.used);
};

int SerialClass::baudRate(){
	return 0;
};

void SerialClass::end(){
	UARTDRV_DeInit(uartHandle);
};

