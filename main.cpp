//////
//////#include "em_device.h"
//////#include "em_chip.h"
//////#include "init_mcu.h"
//////#include "init_board.h"
//////#include "init_app.h"
//////#include "uartdrv.h"
//////#include "hal-config-board.h"
//////#include "uartdrv_config.h"
//////#include "ble-configuration.h"
//////#include "board_features.h"
//////
///////* Bluetooth stack headers */
//////#include "bg_types.h"
//////#include "native_gecko.h"
//////#include "gatt_db.h"
//////
//////#ifndef MAX_CONNECTIONS
//////#define MAX_CONNECTIONS 4
//////#endif
//////uint8_t bluetooth_stack_heap[DEFAULT_BLUETOOTH_HEAP(MAX_CONNECTIONS)];
//////
//////#define EMDRV_UARTDRV_HW_FLOW_CONTROL_ENABLE 0
//////
//////UARTDRV_HandleData_t uarthandleData;
//////UARTDRV_Handle_t uartHandle = &uarthandleData;
//////DEFINE_BUF_QUEUE(EMDRV_UARTDRV_MAX_CONCURRENT_RX_BUFS, rxBufferQueue);
//////DEFINE_BUF_QUEUE(EMDRV_UARTDRV_MAX_CONCURRENT_TX_BUFS, txBufferQueue);
//////uint8_t buffer[64] = "noob sauce\r\n";
//////
//////#define MY_UART                                     \
//////{                                                   \
//////  USART0,                                           \
//////  115200,                                           \
//////  _USART_ROUTELOC0_TXLOC_LOC0,                     \
//////  _USART_ROUTELOC0_RXLOC_LOC0,                     \
//////  usartStopbits1,                                   \
//////  usartNoParity,                                    \
//////  usartOVS16,                                       \
//////  false,                                            \
//////  uartdrvFlowControlNone,							    \
//////  gpioPortA,                                        \
//////  4,                                                \
//////  gpioPortA,                                        \
//////  5,                                                \
//////  (UARTDRV_Buffer_FifoQueue_t *)&rxBufferQueue,     \
//////  (UARTDRV_Buffer_FifoQueue_t *)&txBufferQueue,     \
//////}
////////  _USART_ROUTELOC1_CTSLOC_LOC0,                    \
////////  _USART_ROUTELOC1_RTSLOC_LOC0                     \
//////}
//////
//////static const gecko_configuration_t config = {
//////  .config_flags = 0,
//////  .sleep.flags = 0,
//////  .bluetooth.max_connections = MAX_CONNECTIONS,
//////  .bluetooth.heap = bluetooth_stack_heap,
//////  .bluetooth.heap_size = sizeof(bluetooth_stack_heap),
//////  .bluetooth.sleep_clock_accuracy = 100, // ppm
//////  .gattdb = &bg_gattdb_data,
//////  .ota.flags = 0,
//////  .ota.device_name_len = 3,
//////  .ota.device_name_ptr = "OTA",
//////#if (HAL_PA_ENABLE) && defined(FEATURE_PA_HIGH_POWER)
//////  .pa.config_enable = 1, // Enable high power PA
//////  .pa.input = GECKO_RADIO_PA_INPUT_VBAT, // Configure PA input to VBAT
//////#endif // (HAL_PA_ENABLE) && defined(FEATURE_PA_HIGH_POWER)
//////};
//////
////////  uartdrvFlowControlHwUart
//////
//////void setup_uartdrv(void) {                                      //Setup uartdrv
//////		GPIO_PinOutClear(BSP_VCOM_ENABLE_PORT, BSP_VCOM_ENABLE_PIN);
//////      //Configuration for USART0, Location 1
//////      UARTDRV_Init_t uartInitData = MY_UART;
//////      UARTDRV_Init(uartHandle, &uartInitData);
//////
//////}
//////
//////void callback(UARTDRV_Handle_t handle,
//////              Ecode_t transferStatus,
//////              uint8_t *data,
//////              UARTDRV_Count_t transferCount)
//////{
//////  (void)handle;
//////  (void)transferStatus;
//////  (void)data;
//////  (void)transferCount;
//////	for(volatile long i=0; i<3; i++){
//////		GPIO_PinOutSet(gpioPortF, 6U);
//////		for(volatile long i=0; i<3000000; i++);
//////		GPIO_PinOutClear(gpioPortF, 6U);
//////		for(volatile long i=0; i<3000000; i++);
//////	};
//////}
//////
//////int main(void)
//////{
//////    CHIP_Init();
//////
//////    CMU_ClockEnable(cmuClock_HFPER, true);
//////    CMU_ClockEnable(cmuClock_USART0, true);
//////    CMU_ClockEnable(cmuClock_GPIO, true);
//////	  // Initialize device
//////	  initMcu();
//////	  // Initialize board
//////	  initBoard();
//////	  // Initialize application
//////	  initApp();
//////
//////	   gecko_init(&config);
//////
//////	setup_uartdrv();
//////
//////	GPIO_PinModeSet(gpioPortF, 6U, gpioModePushPull, 0);
//////
//////	for(volatile long i=0; i<3; i++){
//////		GPIO_PinOutSet(gpioPortF, 6U);
//////		for(volatile long i=0; i<3000000; i++);
//////		GPIO_PinOutClear(gpioPortF, 6U);
//////		for(volatile long i=0; i<3000000; i++);
//////	};
//////
//////    // Transmit data using a non-blocking transmit function
////////    UARTDRV_Transmit(uartHandle, buffer, 64, callback);
//////    UARTDRV_Receive(uartHandle, buffer, 64, callback);
//////
//////  while (1) {
//////
//////		// Add some delay
//////
////////		UARTDRV_ForceTransmit(uartHandle, buffer, 64);
////////		for(volatile long i=0; i<3000000; i++);
//////  }
//////}
////
//
//
//#include "em_device.h"
//#include "em_cmu.h"
//#include "em_gpio.h"
////#include "em_usart.h"
//#include "em_chip.h"
//#include "Serial.h"
//
////#define BUFFER_SIZE 80
////volatile uint32_t rx_data_ready = 0;
////volatile char rx_buffer[BUFFER_SIZE];
////volatile char tx_buffer[BUFFER_SIZE];
//
///**************************************************************************//**
// * @brief USART0 RX interrupt service routine
// *****************************************************************************/
////void USART0_RX_IRQHandler(void)
////{
////  static uint32_t i = 0;
////  uint32_t flags;
////  flags = USART_IntGet(USART0);
////  USART_IntClear(USART0, flags);
////
////  /* Store incoming data into rx_buffer, set rx_data_ready when a full
////  * line has been received
////  */
////  rx_buffer[i++] = USART_Rx(USART0);;
////
////  if (rx_buffer[i - 1] == '\r' || rx_buffer[i - 1] == '\f')
////  {
////    rx_data_ready = 1;
////    rx_buffer[i - 1] = '\0'; // Overwrite CR or LF character
////    i = 0;
////  }
////
////  if ( i >= BUFFER_SIZE - 2 )
////  {
////    rx_data_ready = 1;
////    rx_buffer[i] = '\0'; // Do not overwrite last character
////    i = 0;
////  }
////}
////
/////**************************************************************************//**
//// * @brief USART0 TX interrupt service routine
//// *****************************************************************************/
////void USART0_TX_IRQHandler(void)
////{
////  static uint32_t i = 0;
////  uint32_t flags;
////  flags = USART_IntGet(USART0);
////  USART_IntClear(USART0, flags);
////
////  if (flags & USART_IF_TXC)
////  {
////    if (i < BUFFER_SIZE && tx_buffer[i] != '\0')
////    {
////    	USART_Tx(USART0, tx_buffer[i++]);; // Transmit byte
////    }
////    else
////    {
////      i = 0; // No more data to send
////    }
////  }
////}
//
///**************************************************************************//**
// * @brief Main function
// *****************************************************************************/
//int main_void(void)
//{
////  USART_InitAsync_TypeDef init = USART_INITASYNC_DEFAULT;
//  char welcome_string[] = "Silicon Labs TEST Code example!\r\f";
//  uint32_t i;
////
////  // Chip errata
////  CHIP_Init();
////
////  // Enable oscillator to GPIO and USART0 modules
////  CMU_ClockEnable(cmuClock_GPIO, true);
////  CMU_ClockEnable(cmuClock_USART0, true);
////
////  // set pin modes for USART TX and RX pins
////  GPIO_PinModeSet(gpioPortA, 1, gpioModeInput, 0);
////  GPIO_PinModeSet(gpioPortA, 0, gpioModePushPull, 1);
////
////  // Initialize USART asynchronous mode and route pins
////  USART_InitAsync(USART0, &init);
////  USART0->ROUTEPEN |= USART_ROUTEPEN_TXPEN | USART_ROUTEPEN_RXPEN;
////  USART0->ROUTELOC0 = USART_ROUTELOC0_RXLOC_LOC0 | USART_ROUTELOC0_TXLOC_LOC0;
////
////  //Initializae USART Interrupts
////  USART_IntEnable(USART0, USART_IEN_RXDATAV);
////  USART_IntEnable(USART0, USART_IEN_TXC);
////
////  //Enabling USART Interrupts
////  NVIC_EnableIRQ(USART0_RX_IRQn);
////  NVIC_EnableIRQ(USART0_TX_IRQn);
//
//  SerialClass Serial(1);
//
//  Serial.begin(115200);
//
//  // Print welcome message
//  for (i = 0 ; welcome_string[i] != 0; i++)
//  {
//    tx_buffer[i] = welcome_string[i];
//  }
//  USART_IntSet(USART0, USART_IFS_TXC);
//
//  while (1)
//  {
//    /* When notified by the RX handler, copy data from the RX buffer to the
//     * TX buffer, and start the TX handler */
//    if (Serial.available())
//    {
//      USART_IntDisable(USART0, USART_IEN_RXDATAV);
//      USART_IntDisable(USART0, USART_IEN_TXC);
//
//      for (i = 0; rx_buffer[i] != 0 && i < BUFFER_SIZE-3; i++)
//      {
//        tx_buffer[i] = rx_buffer[i];
//      }
//
//      for (i = 0; rx_buffer[i] != 0 && i < BUFFER_SIZE-3; i++)
//      {
//        tx_buffer[i] = rx_buffer[i];
//      }
//
//
//      tx_buffer[i++] = '\r';
//      tx_buffer[i++] = '\f';
//      tx_buffer[i] = '\0';
//      rx_data_ready = 0;
//
//      USART_IntEnable(USART0, USART_IEN_RXDATAV);
//      USART_IntEnable(USART0, USART_IEN_TXC);
//      USART_IntSet(USART0, USART_IFS_TXC);
//    }
//  }
//}
