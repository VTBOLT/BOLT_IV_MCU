#ifndef GLOBALS_INCLUDED
#define GLOBALS_INCLUDED

#include <stdint.h>
#include <stdbool.h>
#include <can_comms.h>

typedef enum ignitState { IGNIT_OFF = 0, IGNIT_ON = 1 } ignitState_t;

/* CAN variables */
extern bool rxMsg;
extern bool errFlag;
extern uint32_t msgCount;

/* Configure system clock for 120 MHz */
extern uint32_t systemClock;

// Timer 1 flag
extern uint8_t intTimer1_flag;

// Debounce counter
extern uint32_t debounceCounter;

// AUX pack voltage
extern uint8_t auxVoltage[voltageLength];


#endif
