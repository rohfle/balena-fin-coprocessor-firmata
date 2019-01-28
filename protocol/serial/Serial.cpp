#include "Serial.h"

SerialClass::SerialClass(unsigned int interface){

	baud = 115200;
	GPIO_PinModeSet(BSP_VCOM_ENABLE_PORT, BSP_VCOM_ENABLE_PIN, gpioModePushPull, 1);
	if(interface == VCOM){
		GPIO_PinOutSet(BSP_VCOM_ENABLE_PORT, BSP_VCOM_ENABLE_PIN);
		uartInitData = VCOM_CONFIG;

	}else{
		GPIO_PinOutClear(BSP_VCOM_ENABLE_PORT, BSP_VCOM_ENABLE_PIN);
		uartInitData = CM3_CONFIG;

	};
};

SerialClass::SerialClass(unsigned int interface, unsigned int baudrate){

	baud = baudrate;
	GPIO_PinModeSet(BSP_VCOM_ENABLE_PORT, BSP_VCOM_ENABLE_PIN, gpioModePushPull, 1);
	if(interface == VCOM){
		GPIO_PinOutSet(BSP_VCOM_ENABLE_PORT, BSP_VCOM_ENABLE_PIN);
		uartInitData = VCOM_CONFIG;
	}else{
		GPIO_PinOutClear(BSP_VCOM_ENABLE_PORT, BSP_VCOM_ENABLE_PIN);
		uartInitData = CM3_CONFIG;
	}
};

void SerialClass::begin(unsigned long baud){
//    UARTDRV_Init_t uartInitData = CM3_CONFIG;
    UARTDRV_Init(uartHandle, &uartInitData);
};

size_t SerialClass::write(unsigned char * c){
	size_t status = UARTDRV_TransmitB(uartHandle, c, (strlen((const char *)c)/sizeof(unsigned char)));
	return status;
};

size_t SerialClass::write(int n){
	unsigned char c [64] = "";
	itoa(n, (char *) c, 10);
	size_t status = UARTDRV_TransmitB(uartHandle, (unsigned char *) c, (sizeof(c)/sizeof(unsigned char)));
	return status;
};

size_t SerialClass::write(int * n){
	unsigned char c [64] = "";
	itoa(*n, (char *) c, 10);
	size_t status = UARTDRV_TransmitB(uartHandle, (unsigned char *) n, (sizeof(n)/sizeof(unsigned char)));
	return status;
};

int SerialClass::read(void){
	uint8_t data[EMDRV_UARTDRV_MAX_CONCURRENT_BUFS];
	UARTDRV_ReceiveB(uartHandle, data, (sizeof(* data)/sizeof(int)));
	return * data;
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

