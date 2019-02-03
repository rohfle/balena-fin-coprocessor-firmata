//#include "Serial.h"
//
//SerialClass::SerialClass(unsigned int interface){
//	GPIO_PinModeSet(BSP_VCOM_ENABLE_PORT, BSP_VCOM_ENABLE_PIN, gpioModePushPull, 1);
//
//	uartInitData.port                 = USART0;
//	uartInitData.baudRate             = 115200;
//	uartInitData.stopBits             = (USART_Stopbits_TypeDef) USART_FRAME_STOPBITS_ONE;
//	uartInitData.parity               = (USART_Parity_TypeDef) USART_FRAME_PARITY_NONE;
//	uartInitData.oversampling         = (USART_OVS_TypeDef) USART_CTRL_OVS_X16;
//	uartInitData.mvdis                = false;
//	uartInitData.fcType               = uartdrvFlowControlNone;
//	uartInitData.rxQueue              = (UARTDRV_Buffer_FifoQueue_t *)&rxBufferQueue;
//	uartInitData.txQueue              = (UARTDRV_Buffer_FifoQueue_t *)&txBufferQueue;
//
//	if(interface == VCOM){
//		GPIO_PinOutSet(BSP_VCOM_ENABLE_PORT, BSP_VCOM_ENABLE_PIN);
////		uartInitData = VCOM_CONFIG;
//
//		uartInitData.portLocationTx       = USART_ROUTELOC0_TXLOC_LOC0;
//		uartInitData.portLocationRx       = USART_ROUTELOC0_RXLOC_LOC0;
//	}else{
//		GPIO_PinOutClear(BSP_VCOM_ENABLE_PORT, BSP_VCOM_ENABLE_PIN);
////		uartInitData = CM3_CONFIG;
//
//		uartInitData.portLocationTx       = USART_ROUTELOC0_TXLOC_LOC0;
//		uartInitData.portLocationRx       = USART_ROUTELOC0_RXLOC_LOC0;
//	};
//};
//
//SerialClass::SerialClass(unsigned int interface, long baudrate){
//	GPIO_PinModeSet(BSP_VCOM_ENABLE_PORT, BSP_VCOM_ENABLE_PIN, gpioModePushPull, 1);
//	if(interface == VCOM){
//		GPIO_PinOutSet(BSP_VCOM_ENABLE_PORT, BSP_VCOM_ENABLE_PIN);
//		uartInitData = VCOM_CONFIG;
//	}else{
//		GPIO_PinOutClear(BSP_VCOM_ENABLE_PORT, BSP_VCOM_ENABLE_PIN);
//		uartInitData = CM3_CONFIG;
//	}
//	uartInitData.baudRate = baudrate;
//};
//
//void SerialClass::begin(long baudrate){
//	GPIO_PinModeSet(BSP_VCOM_ENABLE_PORT, BSP_VCOM_ENABLE_PIN, gpioModePushPull, 1);
//	GPIO_PinOutSet(BSP_VCOM_ENABLE_PORT, BSP_VCOM_ENABLE_PIN);
//	uartInitData = VCOM_CONFIG;
//	uartInitData.baudRate = baudrate;
////		GPIO_PinOutClear(BSP_VCOM_ENABLE_PORT, BSP_VCOM_ENABLE_PIN);
////		uartInitData = CM3_CONFIG;
//    UARTDRV_Init(uartHandle, &uartInitData);
//    USART_IntEnable(USART0, USART_IEN_RXDATAV);
////    NVIC_EnableIRQ(USART0_RX_IRQn);
//};
//
//size_t SerialClass::write(unsigned char * c){
//	size_t status = UARTDRV_TransmitB(uartHandle, c, (strlen((const char *)c)/sizeof(unsigned char)));
//	return status;
//};
//
//size_t SerialClass::write(int n){
//	unsigned char c [64] = {(char) n};
//	size_t status = UARTDRV_TransmitB(uartHandle, (unsigned char *) c, (sizeof(c)/sizeof(unsigned char)));
//	return status;
//};
//
//size_t SerialClass::write(int * n){
//	unsigned char c [64] = {(char) * n};
//	size_t status = UARTDRV_TransmitB(uartHandle, (unsigned char *) c, (sizeof(c)/sizeof(unsigned char)));
//	return status;
//};
//
//int SerialClass::read(void){
////	UARTDRV_ReceiveB(uartHandle, rx_buffer, (sizeof(* rx_buffer)/sizeof(char)));
//	Ecode_t ecode = UARTDRV_Receive(uartHandle, rx_buffer, (sizeof(* rx_buffer)/sizeof(char)), uartRxCallback);
//	if(ecode != ECODE_EMDRV_UARTDRV_OK){
//		while(1);
//}
//	return * rx_buffer;
//};
//
//void SerialClass::uartRxCallback(UARTDRV_Handle_t handle,
//              Ecode_t transferStatus,
//              uint8_t *data,
//              UARTDRV_Count_t transferCount)
//{
//				digitalWrite(3,1);
//				delay(1000);
//				digitalWrite(3,0);
//				delay(1000);
//}
//
//size_t SerialClass::readBytes(uint8_t* buffer, size_t size){
//	size_t status = UARTDRV_ReceiveB(uartHandle,buffer,size);
//	return status;
//};
//
//int SerialClass::available(void){
////	uint8_t data = 0;
////	UARTDRV_Status_t count = UARTDRV_GetReceiveDepth(uartHandle);
////	if(count==0) return -1;
////	else return count;
//
////	if(rx_data_ready) return 1;
////	else return 0;
//
////	UARTDRV_Status_t status = UARTDRV_GetReceiveStatus(uartHandle, &RXBuffer, &RXReceived, &RXRemaining);
////	UARTDRV_Status_t status = UARTDRV_GetPeripheralStatus(uartHandle);
////	data = USART_IntGetEnabled(USART0);
////	data =  USART_IntGet(uartInitData.port) & USART_IEN_RXDATAV;
//	return USART_RxDataGet(USART0);
////	return UARTDRV_GetPeripheralStatus(uartHandle);
////	if(USART0->STATUS & UARTDRV_STATUS_RXDATAV) return 1;
////	else return 0;
//};
//
//int SerialClass::availableForWrite(void){
//	return (txBufferQueue.size - txBufferQueue.used);
//};
//
//int SerialClass::baudRate(){
//	return 0;
//};
//
//void SerialClass::end(){
//	UARTDRV_DeInit(uartHandle);
//};
//

//#include "Serial.h"
//
//volatile char rx_buffer[BUFFER_SIZE];
//volatile char tx_buffer[BUFFER_SIZE];
//volatile uint32_t rx_data_ready;
//volatile uint32_t rx_data_available;
//volatile uint32_t tx_data_available;
//
//
//void USART0_TX_IRQHandler(void)
//{
//  static uint32_t i = 0;
//  uint32_t flags;
//  flags = USART_IntGet(USART0);
//  USART_IntClear(USART0, flags);
//
//  if (flags & USART_IF_TXC)
//  {
//    if (i < BUFFER_SIZE && tx_buffer[i] != '\0')
//    {
//    	USART_Tx(USART0, tx_buffer[i++]);; // Transmit byte
////		tx_buffer[i] = 0;
//    }
//    else
//    {
//      i = 0; // No more data to send
//    }
//  }
//  tx_data_available = i; // set tx_data_available = the number of chars in buffer
//}
//
//void USART0_RX_IRQHandler(void)
//{
//  static uint32_t i = 0;
//  uint32_t flags;
//  flags = USART_IntGet(USART0);
//  USART_IntClear(USART0, flags);
//
//  /* Store incoming data into rx_buffer, set rx_data_ready when a full
//  * line has been received
//  */
//  rx_buffer[i++] = USART_Rx(USART0);;
//  rx_data_available += i; // set rx_data_available = the number of chars in buffer
//
//
//  if (rx_buffer[i - 1] == '\r' || rx_buffer[i - 1] == '\f')
//  {
//    rx_data_ready = 1;
//    rx_buffer[i - 1] = '\0'; // Overwrite CR or LF character
//    i = 0;
//  }
//
//  if ( i >= BUFFER_SIZE - 2 )
//  {
//    rx_data_ready = 1;
//    rx_buffer[i] = '\0'; // Do not overwrite last character
//    i = 0;
//  }
////  rx_data_available = i; // set rx_data_available = the number of chars in buffer
//}
//
//SerialClass::SerialClass(unsigned int interface){
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
//};
//
//SerialClass::SerialClass(unsigned int interface, long baudrate){
//
//};
//
//void SerialClass::begin(long baudrate){
//	init.baudrate = baudrate;
//
//	  //Initializae USART Interrupts
//	  USART_IntEnable(USART0, USART_IEN_RXDATAV);
//	  USART_IntEnable(USART0, USART_IEN_TXC);
//
//	  //Enabling USART Interrupts
//	  NVIC_EnableIRQ(USART0_RX_IRQn);
//	  NVIC_EnableIRQ(USART0_TX_IRQn);
//};
//
//size_t SerialClass::write(unsigned char * c){
//size_t i = 0;
//  for (i = 0 ; c[i] != 0; i++)
//  {
//	tx_buffer[i] = c[i];
//  }
//  USART_IntSet(USART0, USART_IFS_TXC);
//
//  return i;
//};
//
//size_t SerialClass::write(int n){
//	tx_buffer[0] = n;
//	USART_IntSet(USART0, USART_IFS_TXC);
//	return sizeof(n);
//};
//
//size_t SerialClass::write(int * n){
//	size_t i = 0;
//	  for (i = 0 ; n[i] != 0; i++)
//	  {
//		tx_buffer[i] = n[i];
//	  }
//	  USART_IntSet(USART0, USART_IFS_TXC);
//
//	  return i;
//};
//
//int SerialClass::read(void){
////	unsigned char myarray[BUFFER_SIZE];
//	        char temp;
//	        //for loop to print the array with indexes moved up (to the left) <-- by 2
//	        for (int i=0; i < BUFFER_SIZE-1; i++)
//	        {//EXAMPLE shift by 3  for a c-string of 5
//	            temp = rx_buffer[i];//temp = myarray[0]
//	            rx_buffer[i] = rx_buffer[i + 1];//myarray[0] == myarray[2]
//	            rx_buffer[i + 1] = temp;//myarray[2] = temp(value previously at index i)
//	        }
//	return rx_buffer[0];
////	return -1;
//};
//
//size_t SerialClass::readBytes(uint8_t* buffer, size_t size){
//    USART_IntDisable(USART0, USART_IEN_RXDATAV);
//    USART_IntDisable(USART0, USART_IEN_TXC);
//	size_t i = 0;
//	for (i = 0; i < size - 1 ; i++ )
//	{
//	  if (rx_buffer[i - 1] == '\r' || rx_buffer[i - 1] == '\f')
//	  {
//		break; // Breaking on CR prevents it from being counted in the number of bytes
//	  }
//	buffer[i] = rx_buffer[i];
//	rx_buffer[i] = 0;
//	}
//	rx_data_available = 0;
//    USART_IntEnable(USART0, USART_IEN_RXDATAV);
//    USART_IntEnable(USART0, USART_IEN_TXC);
//    USART_IntSet(USART0, USART_IFS_TXC);
//    if(i==0) return -1;
//    else return i;
//};
//
//int SerialClass::available(void){
////	return strlen((const char *)rx_buffer);
//	return rx_data_ready;
//
//};
//
//int SerialClass::availableForWrite(void){
//	return tx_data_available;
//};
//
//int SerialClass::baudRate(){
//	return init.baudrate;
//};
//
//void SerialClass::end(){
////	UARTDRV_DeInit(uartHandle);
//};
//
//

/******************************************************************************************************************/

#include "Serial.h"

const char     welcomeString[]  = "EFM32 RS-232 - Please press a key\n";
const char     overflowString[] = "\n---RX OVERFLOW---\n";
const uint32_t welLen           = sizeof(welcomeString) - 1;
const uint32_t ofsLen           = sizeof(overflowString) - 1;

/* Declare a circular buffer structure to use for Rx and Tx queues */
#define BUFFERSIZE          256

struct circularBuffer
{
  uint8_t  data[BUFFERSIZE];  /* data buffer */
  uint32_t rdI;               /* read index */
  uint32_t wrI;               /* write index */
  uint32_t pendingBytes;      /* count of how many bytes are not yet handled */
  bool     overflow;          /* buffer overflow indicator */
} ;

volatile circularBuffer rxBuf;
volatile circularBuffer txBuf = { {0}, 0, 0, 0, false };


/**************************************************************************//**
 * @brief UART0 RX IRQ Handler
 *
 * Set up the interrupt prior to use
 *
 *****************************************************************************/

void USART0_RX_IRQHandler(void)
{
  /* Check for RX data valid interrupt */
  if (USART0->IF & USART_IF_RXDATAV)
  {
    /* Copy data into RX Buffer */
    uint8_t rxData = USART_Rx(USART0);
    rxBuf.data[rxBuf.wrI] = rxData;
    rxBuf.wrI             = (rxBuf.wrI + 1) % BUFFERSIZE;
    rxBuf.pendingBytes++;

    /* Flag Rx overflow */
    if (rxBuf.pendingBytes > BUFFERSIZE)
    {
      rxBuf.overflow = true;
    }
  }
}

/**************************************************************************//**
 * @brief UART0 TX IRQ Handler
 *
 * Set up the interrupt prior to use
 *
 *****************************************************************************/
void USART0_TX_IRQHandler(void)
{
  /* Check TX buffer level status */
  if (USART0->IF & USART_IF_TXBL)
  {
    if (txBuf.pendingBytes > 0)
    {
      /* Transmit pending character */
      USART_Tx(USART0, txBuf.data[txBuf.rdI]);
      txBuf.rdI = (txBuf.rdI + 1) % BUFFERSIZE;
      txBuf.pendingBytes--;
    }

    /* Disable Tx interrupt if no more bytes in queue */
    if (txBuf.pendingBytes == 0)
    {
      USART_IntDisable(USART0, USART_IEN_TXBL);
    }
  }
}

SerialClass::SerialClass(){
	 /* Enable clock for GPIO module (required for pin configuration) */
	  /* Enable clock for HF peripherals */
	  CMU_ClockEnable(cmuClock_HFPER, true);

	  /* Enable clock for USART module */
	  CMU_ClockEnable(cmuClock_USART0, true);
	  CMU_ClockEnable(cmuClock_GPIO, true);
	  /* Configure GPIO pins */
	  GPIO_PinModeSet(gpioPortA, 1, gpioModeInput, 0);
	  GPIO_PinModeSet(gpioPortA, 0, gpioModePushPull, 1);


	  /* Prepare struct for initializing UART in asynchronous mode*/
	  uartInit.enable       = usartDisable;   /* Don't enable UART upon intialization */
	  uartInit.refFreq      = 0;              /* Provide information on reference frequency. When set to 0, the reference frequency is */
	  uartInit.baudrate     = 115200;         /* Baud rate */
	  uartInit.oversampling = usartOVS16;     /* Oversampling. Range is 4x, 6x, 8x or 16x */
	  uartInit.databits     = usartDatabits8; /* Number of data bits. Range is 4 to 10 */
	  uartInit.parity       = usartNoParity;  /* Parity mode */
	  uartInit.stopbits     = usartStopbits1; /* Number of stop bits. Range is 0 to 2 */
	  uartInit.mvdis        = false;          /* Disable majority voting */
	  uartInit.prsRxEnable  = false;          /* Enable USART Rx via Peripheral Reflex System */
	  uartInit.prsRxCh      = usartPrsRxCh0;  /* Select PRS channel if enabled */


};

SerialClass::SerialClass(unsigned int interface, long baudrate){

};

void SerialClass::begin(long baudrate){
	uartInit.baudrate = baudrate;
	  /* Initialize USART with uartInit struct */
	  USART_InitAsync(USART0, &uartInit);

	  /* Prepare UART Rx and Tx interrupts */
	  USART_IntClear(USART0, _USART_IFC_MASK);
	  USART_IntEnable(USART0, USART_IEN_RXDATAV);
	  NVIC_ClearPendingIRQ(USART0_RX_IRQn);
	  NVIC_ClearPendingIRQ(USART0_TX_IRQn);
	  NVIC_EnableIRQ(USART0_RX_IRQn);
	  NVIC_EnableIRQ(USART0_TX_IRQn);

	  /* Enable I/O pins at UART1 location #2 */
	  USART0->ROUTEPEN |= USART_ROUTEPEN_TXPEN | USART_ROUTEPEN_RXPEN;
	  USART0->ROUTELOC0 = USART_ROUTELOC0_RXLOC_LOC0 | USART_ROUTELOC0_TXLOC_LOC0;
	  /* Enable UART */
	USART_Enable(USART0, usartEnable);
};

void SerialClass::write(uint8_t ch){
	  /* Check if Tx queue has room for new data */
	  if ((txBuf.pendingBytes + 1) > BUFFERSIZE)
	  {
	    /* Wait until there is room in queue */
	    while ((txBuf.pendingBytes + 1) > BUFFERSIZE) ;
	  }

	  /* Copy ch into txBuffer */
	  txBuf.data[txBuf.wrI] = ch;
	  txBuf.wrI             = (txBuf.wrI + 1) % BUFFERSIZE;

	  /* Increment pending byte counter */
	  txBuf.pendingBytes++;

	  /* Enable interrupt on USART TX Buffer*/
	  USART_IntEnable(USART0, USART_IEN_TXBL);
};

void SerialClass::write(uint8_t * dataPtr, uint32_t dataLen)
{
  uint32_t i = 0;



  /* Check if buffer is large enough for data */
  if (dataLen > BUFFERSIZE)
  {
    /* Buffer can never fit the requested amount of data */
    return;
  }

  /* Check if buffer has room for new data */
  if ((txBuf.pendingBytes + dataLen) > BUFFERSIZE)
  {
    /* Wait until room */
    while ((txBuf.pendingBytes + dataLen) > BUFFERSIZE) ;
  }

  /* Fill dataPtr[0:dataLen-1] into txBuffer */
  while (i < dataLen)
  {
    txBuf.data[txBuf.wrI] = *(dataPtr + i);
    txBuf.wrI             = (txBuf.wrI + 1) % BUFFERSIZE;
    i++;
  }

  /* Increment pending byte counter */
  txBuf.pendingBytes += dataLen;

  /* Enable interrupt on USART TX Buffer*/
  USART_IntEnable(USART0, USART_IEN_TXBL);
}

int SerialClass::read(void)
{
  uint8_t ch;
  /* Check if there is a byte that is ready to be fetched. If no byte is ready, wait for incoming data */
  if (rxBuf.pendingBytes < 1)
  {
//    while (rxBuf.pendingBytes < 1) ;
	return -1;
  }

  /* Copy data from buffer */
  ch        = rxBuf.data[rxBuf.rdI];
  rxBuf.rdI = (rxBuf.rdI + 1) % BUFFERSIZE;

  /* Decrement pending byte counter */
  rxBuf.pendingBytes--;

  return ch;
};

uint32_t SerialClass::readBytes(uint8_t * dataPtr, uint32_t dataLen)
{
  uint32_t i = 0;

  /* Wait until the requested number of bytes are available */
  if (rxBuf.pendingBytes < dataLen)
  {
    while (rxBuf.pendingBytes < dataLen) ;
  }

  if (dataLen == 0)
  {
    dataLen = rxBuf.pendingBytes;
  }

  /* Copy data from Rx buffer to dataPtr */
  while (i < dataLen)
  {
    *(dataPtr + i) = rxBuf.data[rxBuf.rdI];
    rxBuf.rdI      = (rxBuf.rdI + 1) % BUFFERSIZE;
    i++;
  }

  /* Decrement pending byte counter */
  rxBuf.pendingBytes -= dataLen;

  return i;
}

int SerialClass::available(void){
	if(rxBuf.pendingBytes == 0) return -1;
	else return rxBuf.pendingBytes;

};

int SerialClass::availableForWrite(void){
	if(txBuf.pendingBytes == 0) return -1;
	else return txBuf.pendingBytes;
};

int SerialClass::baudRate(){
	return uartInit.baudrate;
};

void SerialClass::end(){
//	UARTDRV_DeInit(uartHandle);
};

