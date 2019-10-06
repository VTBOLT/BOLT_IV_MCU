#include "msp.h"

/* Standard driverlib include - can be more specific if needed */
#include <globals.h>
#include <ti/devices/msp432e4/driverlib/driverlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "uartstdio.h"

#include "can_comms.h"
#include "uart_comms.h"

bool rxMsg = false;
bool errFlag = false;
uint32_t msgCount = 0;

/* Configure system clock for 120 MHz */
uint32_t systemClock;

// Timer 1 flag
uint8_t intTimer1_flag = 0;

// Debounce counter
uint32_t debounceCounter = 0;
