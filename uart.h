#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ti/devices/msp432e4/driverlib/driverlib.h>

#include "uartstdio.h"

#define IMU_RECEIVE_BAUD 57600

void UARTSendChar(uint32_t, const char);
void UARTSendCharNonBlocking(uint32_t, const char);
void UARTSendStr(uint32_t, const uint8_t*, uint32_t);
void UARTSendStrNonBlocking(uint32_t, const uint8_t*, uint32_t);
void UART7Setup(uint32_t);
void UART6Setup(uint32_t);
void enableUARTprintf(uint32_t);
