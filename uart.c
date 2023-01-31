/*
 * uart.c
 *
 *  Created on: Jan 31, 2023
 *      Author: Molly Shear
 */

#include "uart.h"


void UARTSendChar(uint32_t UART_BASE, const char c) {
  MAP_UARTCharPut(UART_BASE, c);
}

void UARTSendCharNonBlocking(uint32_t UART_BASE, const char c) {
  MAP_UARTCharPutNonBlocking(UART_BASE, c);
}

void UARTSendStr(uint32_t UART_BASE, const uint8_t* pui8Buffer,
                 uint32_t ui32Count) {
  //
  // Loop while there are more characters to send.
  //
  while (ui32Count--) {
    //
    // Write the next character to the UART.
    //
    MAP_UARTCharPut(UART_BASE, *pui8Buffer++);
  }
}

void UARTSendStrNonBlocking(uint32_t UART_BASE, const uint8_t* pui8Buffer,
                            uint32_t ui32Count) {
  //
  // Loop while there are more characters to send.
  //
  while (ui32Count--) {
    //
    // Write the next character to the UART.
    //
    MAP_UARTCharPutNonBlocking(UART_BASE, *pui8Buffer++);
  }
}

void UART6Setup(uint32_t systemClock) {
  /* UART Transmit Setup */

  /* Enable clock to peripherals used */
  MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART6);
  MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOP);

  /* Set PP0 and PP1 as UART pins */
  GPIOPinConfigure(GPIO_PP0_U6RX);
  GPIOPinConfigure(GPIO_PP1_U6TX);
  MAP_GPIOPinTypeUART(GPIO_PORTP_BASE, GPIO_PIN_0 | GPIO_PIN_1);

  /* Configure UART for 57,600, 8-N-1 */
  MAP_UARTConfigSetExpClk(
      UART6_BASE, systemClock, IMU_RECEIVE_BAUD,
      UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE);

  MAP_IntEnable(INT_UART6);
  MAP_UARTIntEnable(UART6_BASE, UART_INT_RX | UART_INT_RT);
}

void UART7Setup(uint32_t systemClock) {
  /* UART Transmit Setup */

  /* Enable clock to peripherals used */
  MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART7);
  MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);

  /* Set PC4 and PC5 as UART pins */
  GPIOPinConfigure(GPIO_PC4_U7RX);
  GPIOPinConfigure(GPIO_PC5_U7TX);
  MAP_GPIOPinTypeUART(GPIO_PORTC_BASE, GPIO_PIN_4 | GPIO_PIN_5);

  /* Configure UART for 57,600, 8-N-1 */
  MAP_UARTConfigSetExpClk(
      UART7_BASE, systemClock, 57600,
      UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE);
}

void enableUARTprintf(uint32_t systemClock) {
  /* Enable the GPIO Peripheral used by the UART */
  MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
  while (!(MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOD))) {
  }

  /* Enable UART2 */
  MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART2);

  /* Configure GPIO Pins for UART mode */
  MAP_GPIOPinConfigure(GPIO_PD4_U2RX);
  MAP_GPIOPinConfigure(GPIO_PD5_U2TX);
  MAP_GPIOPinTypeUART(GPIO_PORTD_BASE, GPIO_PIN_4 | GPIO_PIN_5);

  /* Initialize the UART for console I/O */
  UARTStdioConfig(2, 115200, systemClock);
}

void UART6_IRQHandler(void) {
  // UARTprintf("Entered UART6 ISR\n");
  uint32_t ui32Status;

  //
  // Get the interrrupt status.
  //
  ui32Status = MAP_UARTIntStatus(UART6_BASE, true);

  //
  // Clear the asserted interrupts.
  //
  MAP_UARTIntClear(UART6_BASE, ui32Status);

  //
  // Loop while there are characters in the receive FIFO.
  //
  while (MAP_UARTCharsAvail(UART6_BASE)) {
    char c = MAP_UARTCharGetNonBlocking(UART6_BASE);
    // UARTprintf(c);
    // MAP_UARTCharPutNonBlocking(UART0_BASE, c);
    imuParse(c);
  }
}
