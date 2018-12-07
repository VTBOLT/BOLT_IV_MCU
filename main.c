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

int main(void)
{
    // Enable the clock to GPIO Port N and wait for it to be ready
    // The clock must be enabled to any GPIO port being used

    uint32_t allSysPorts[GPIO_PORTS] = {SYSCTL_PERIPH_GPIOA, SYSCTL_PERIPH_GPIOB, SYSCTL_PERIPH_GPIOC, SYSCTL_PERIPH_GPIOD,
                         SYSCTL_PERIPH_GPIOE, SYSCTL_PERIPH_GPIOF, SYSCTL_PERIPH_GPIOG, SYSCTL_PERIPH_GPIOH,
                         SYSCTL_PERIPH_GPIOJ, SYSCTL_PERIPH_GPIOK, SYSCTL_PERIPH_GPIOL, SYSCTL_PERIPH_GPIOM,
                         SYSCTL_PERIPH_GPION, SYSCTL_PERIPH_GPIOP, SYSCTL_PERIPH_GPIOQ};

    uint32_t allGPIOPorts[GPIO_PORTS] = {GPIO_PORTA_BASE, GPIO_PORTB_BASE, GPIO_PORTC_BASE, GPIO_PORTD_BASE,
                                         GPIO_PORTE_BASE, GPIO_PORTF_BASE, GPIO_PORTG_BASE, GPIO_PORTH_BASE,
                                         GPIO_PORTJ_BASE, GPIO_PORTK_BASE, GPIO_PORTL_BASE, GPIO_PORTM_BASE,
                                         GPIO_PORTN_BASE, GPIO_PORTP_BASE, GPIO_PORTQ_BASE};

    int i;
    for (i = 0; i < GPIO_PORTS; i++) {
        MAP_SysCtlPeripheralEnable(allSysPorts[i]);
        while(!(MAP_SysCtlPeripheralReady(allSysPorts[i]))) {};
    }

    uint32_t allPins = (GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7);

    // Configure pins being used (PN5 and PN4) for output
    for (i = 0; i < GPIO_PORTS; i++) {
        MAP_GPIOPinTypeGPIOOutput(allGPIOPorts[i], allPins);
        MAP_GPIOPinWrite(allGPIOPorts[i], allPins, allPins);
    }

    // Write values to each pin
    // param 1: the port
    // param 2: the pins to be set
    // param 3: the value (0 or 1), at bit-specific position
    MAP_GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_0, GPIO_PIN_0);

    while (1)
    {
        // a horrible way to switch LED on and off
        int i;
        for (i=0; i<1000000; i++) {}
        MAP_GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_0, ~(MAP_GPIOPinRead(GPIO_PORTN_BASE, GPIO_PIN_0)));
    }
}
