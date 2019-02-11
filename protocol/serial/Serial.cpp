#include "Serial.h"

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
 * RX IRQ
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
 * TX IRQ
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

/**************************************************************************//**
 * @brief SerialClass Constructor
 *
 *
 *
 *****************************************************************************/

SerialClass::SerialClass(){
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
	// TODO
};

/**************************************************************************//**
 * @brief SerialClass Begin
 *
 * Enables UART and UART Interrupts
 *
 *****************************************************************************/

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

/**************************************************************************//**
 * @brief SerialClass Begin
 *
 * Enables UART and UART Interrupts
 *
 *****************************************************************************/

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

/**************************************************************************//**
 * @brief SerialClass Begin
 *
 * Enables UART and UART Interrupts
 *
 *****************************************************************************/

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

