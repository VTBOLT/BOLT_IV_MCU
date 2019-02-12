/*****************************************************************************
*
* Copyright (C) 2013 - 2017 Texas Instruments Incorporated - http://www.ti.com/
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions
* are met:
*
* * Redistributions of source code must retain the above copyright
*   notice, this list of conditions and the following disclaimer.
*
* * Redistributions in binary form must reproduce the above copyright
*   notice, this list of conditions and the following disclaimer in the
*   documentation and/or other materials provided with the
*   distribution.
*
* * Neither the name of Texas Instruments Incorporated nor the names of
*   its contributors may be used to endorse or promote products derived
*   from this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
* A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
* OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
* SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
* LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
* DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
* THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
******************************************************************************
*
* MSP432 empty main.c template
*
******************************************************************************
*
* This file is the empty main for the BOLT MCU. See can_example.c for details
* when implementing CAN functionality.
*
******************************************************************************
* Potentially useful functions:
* GPIOPinTypeADC(uint32_t ui32Port, uint8_t ui8Pins)
* GPIOPinTypeCAN(uint32_t ui32Port, uint8_t ui8Pins)
* GPIOPinTypeGPIOInput(uint32_t ui32Port, uint8_t ui8Pins)
* GPIOPinTypeGPIOOutput(uint32_t ui32Port, uint8_t ui8Pins)
*
******************************************************************************
* Notes:
* Must include "MAP_" before function calls, I don't know why, but otherwise
*   it doesn't actually see the function and doesn't compile.
*
******************************************************************************/


#include "msp.h"

/* Standard driverlib include - can be more specific if needed */
#include <ti/devices/msp432e4/driverlib/driverlib.h>

#include <stdint.h>
#include <stdbool.h>

#define GPIO_PORTS 15

// State and event names
typedef enum states { PREPUMP, PREIGNIT, POSTIGNIT, MAX_STATES } states;

// Forward Declarations
void IgnitAccessEnable(void);
uint32_t IgnitPoll(void);
uint32_t AccessPoll(void);

// State Subroutines
states prePump (void);
states preIgnit (void);
states postIgnit (void);

// Lookup table for subroutine of a state
states (*const state_array [MAX_STATES]) (void) = { prePump, preIgnit, postIgnit };

int main(void)
{
    // delete
    uint32_t allSysPorts[GPIO_PORTS] = {SYSCTL_PERIPH_GPIOA, SYSCTL_PERIPH_GPIOB, SYSCTL_PERIPH_GPIOC, SYSCTL_PERIPH_GPIOD,
                                        SYSCTL_PERIPH_GPIOE, SYSCTL_PERIPH_GPIOF, SYSCTL_PERIPH_GPIOG, SYSCTL_PERIPH_GPIOH,
                                        SYSCTL_PERIPH_GPIOJ, SYSCTL_PERIPH_GPIOK, SYSCTL_PERIPH_GPIOL, SYSCTL_PERIPH_GPIOM,
                                        SYSCTL_PERIPH_GPION, SYSCTL_PERIPH_GPIOP, SYSCTL_PERIPH_GPIOQ};

    int i = 0;
    for (i = 0; i < GPIO_PORTS; ++i) {
        MAP_SysCtlPeripheralEnable(allSysPorts[i]);
        while(!MAP_SysCtlPeripheralReady(allSysPorts[i])){};
    }

    IgnitAccessEnable();

    states cur_state = PREPUMP;

    // Loop forever.
    while (1)
    {

        if( (cur_state >= 0) && (cur_state < MAX_STATES) ) {
            cur_state = state_array[cur_state]();
        }

    }
}


// Enables all MCU pins involved in the ignition and accessory switches
void IgnitAccessEnable(void)
{
    // Enable the GPIO Pins for Ignition and Accessory Tx as outputs.
    // (Pins PA7 and PM7, respectively) Then, set them to HIGH.
    MAP_GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, GPIO_PIN_7);
    MAP_GPIOPinTypeGPIOOutput(GPIO_PORTM_BASE, GPIO_PIN_7);
    MAP_GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_7, GPIO_PIN_7);
    MAP_GPIOPinWrite(GPIO_PORTM_BASE, GPIO_PIN_7, GPIO_PIN_7);

    // Enable the GPIO Pins for the Ignition and Accessory relays as outputs.
    // (Pins PH1 and PK6, respectively) Then, set them to LOW.
    MAP_GPIOPinTypeGPIOOutput(GPIO_PORTH_BASE, GPIO_PIN_1);
    MAP_GPIOPinTypeGPIOOutput(GPIO_PORTK_BASE, GPIO_PIN_6);
    MAP_GPIOPinWrite(GPIO_PORTH_BASE, GPIO_PIN_1, ~GPIO_PIN_1);
    MAP_GPIOPinWrite(GPIO_PORTK_BASE, GPIO_PIN_6, ~GPIO_PIN_6);

    // Enable the GPIO pin for Ignition and Accessory Rx as inputs.
    // (Pins PP3 and PP5, respectively)
    // Then, enable the MCU's pull-down resistors on each.
    MAP_GPIOPinTypeGPIOInput(GPIO_PORTP_BASE, (GPIO_PIN_3 | GPIO_PIN_5));
    GPIOP->PDR |= (GPIO_PIN_3 | GPIO_PIN_5);
}


// Poll if ignition Rx is HIGH. If so, output HIGH to ignition relay.
// Else, keep output to ignition relay LOW.
// Return state of ignition Rx as bit packed byte
uint32_t IgnitPoll(void)
{
    uint32_t input = MAP_GPIOPinRead(GPIO_PORTP_BASE, GPIO_PIN_3);

    // Ignition switch, Rx: PP3, Relay Output: PH1
    if ( input == GPIO_PIN_3) {
        MAP_GPIOPinWrite(GPIO_PORTH_BASE, GPIO_PIN_1, GPIO_PIN_1);
    } else {
        MAP_GPIOPinWrite(GPIO_PORTH_BASE, GPIO_PIN_1, ~(GPIO_PIN_1));
    }

    return input;

}


// Poll if accessory Rx is HIGH. If so, output HIGH to accessory relay.
// Else, keep output to accessory relay LOW.
// Return state of accessory Rx as bit packed byte
uint32_t AccessPoll(void)
{
    uint32_t input = MAP_GPIOPinRead(GPIO_PORTP_BASE, GPIO_PIN_5);

    // Accessory switch, Rx: PP5, Relay Output: PK6
    if (input == GPIO_PIN_5) {
        MAP_GPIOPinWrite(GPIO_PORTK_BASE, GPIO_PIN_6, GPIO_PIN_6);
    } else {
        MAP_GPIOPinWrite(GPIO_PORTK_BASE, GPIO_PIN_6, ~(GPIO_PIN_6));
    }

    return input;

}

// Subroutine for PREPUMP state.
// Checks if accessory switch is closed.
// Changes relay output if so and returns next state.
states prePump (void)
{
    uint32_t accessStatus = MAP_GPIOPinRead(GPIO_PORTP_BASE, GPIO_PIN_5);
    while(accessStatus != GPIO_PIN_5)
    {
       accessStatus = AccessPoll();
    }

    return PREIGNIT;
}

// Subroutine for PREIGNIT state.
// Checks if accessory switch is opened or ignition switch is close.
// Changes relay output in response to one of the events and returns next state.
states preIgnit (void)
{
    uint32_t accessStatus = MAP_GPIOPinRead(GPIO_PORTP_BASE, GPIO_PIN_5);
    uint32_t ignitStatus = MAP_GPIOPinRead(GPIO_PORTP_BASE, GPIO_PIN_3);

    // while ACCESS ON, IGNIT OFF
    while(accessStatus == GPIO_PIN_5 && ignitStatus == ~GPIO_PIN_3)
    {
        accessStatus = AccessPoll();
        ignitStatus = IgnitPoll();
    }

    // Decides which state to return
    // One of 4 combinations of access Rx and ignit Rx
    // GPIO_PIN_5 is  00001000 and GPIO_PIN_3 00000010
    switch (accessStatus | ignitStatus )
    {
        // ACCESS ON, IGNIT ON
        case (GPIO_PIN_5 | GPIO_PIN_3):
            return POSTIGNIT;
            break;

        // ACCESS OFF, IGNIT OFF
        case (~GPIO_PIN_5 | ~GPIO_PIN_3):
            return PREPUMP;
            break;

        // ACCESS OFF, IGNIT ON (pump must be on,
        default:
            return PREPUMP;
            break;
    }
}

