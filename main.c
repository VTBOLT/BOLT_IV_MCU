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
* MSP432 BOLT IV MCU PROGRAM
*
* Authors:
* William Campbell
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
* Must include "MAP_" before driverlib function calls, I don't know why, but otherwise
*   it doesn't actually see the function and doesn't compile.
*
******************************************************************************/

#include "msp.h"

/* Standard driverlib include - can be more specific if needed */
#include <ti/devices/msp432e4/driverlib/driverlib.h>

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

uint32_t systemClock;

// Function prototypes
void UARTSend(const uint8_t *pui8Buffer, uint32_t ui32Count);
void auxADCSetup();
uint32_t auxADCSend(uint32_t* auxBatVoltage);
void UART7Setup();

// State and event names
typedef enum states { PREPUMP, PREIGNIT, POSTIGNIT, MAX_STATES } states;

// Forward Declarations
void switchesSetup(void);
uint32_t ignitionPoll(void);
uint32_t accessoryPoll(void);

// State Subroutines
states prePump (void);
states preIgnit (void);
states postIgnit (void);

// Lookup array for subroutine of a state
states (*const state_array [MAX_STATES]) (void) = { prePump, preIgnit, postIgnit };

int main(void)
{
    /* Configure system clock for 120 MHz */
    systemClock = MAP_SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ | SYSCTL_OSC_MAIN |
                                          SYSCTL_USE_PLL | SYSCTL_CFG_VCO_480),
                                          120000000);

    uint32_t auxBatVoltage[1];
    uint32_t auxBatAdjusted; //no decimal, accurate value
    states cur_state = PREPUMP;

    auxADCSetup();
    UART7Setup();
    switchesSetup();


    // Loop forever.
    while (1)
    {
        auxBatAdjusted = auxADCSend(auxBatVoltage); // what does this do?

        if(cur_state < MAX_STATES) {
            cur_state = state_array[cur_state]();
        }

        MAP_SysCtlDelay(systemClock/2); // what does this do?
    }
}

void UARTSend(const uint8_t *pui8Buffer, uint32_t ui32Count)
{
    //
    // Loop while there are more characters to send.
    //
    while(ui32Count--)
    {
        //
        // Write the next character to the UART.
        //
        MAP_UARTCharPutNonBlocking(UART7_BASE, *pui8Buffer++);
    }
}


void switchesSetup(void)
{
    /* Enables all MCU pins involved in the ignition and accessory switches */


    /* Enable clock peripherals used
    (A, M, H and K) */
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    while(!MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOA)){};
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOM);
    while(!MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOM)){};
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOH);
    while(!MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOH)){};
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOK);
    while(!MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOK)){};

    /*Enable the GPIO Pins for Ignition and Accessory Tx as outputs.
    ( PA7 and PM7, respectively) */
    MAP_GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, GPIO_PIN_7);
    MAP_GPIOPinTypeGPIOOutput(GPIO_PORTM_BASE, GPIO_PIN_7);

    /* Set them to HIGH */
    MAP_GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_7, GPIO_PIN_7);
    MAP_GPIOPinWrite(GPIO_PORTM_BASE, GPIO_PIN_7, GPIO_PIN_7);

    /* Enable the GPIO Pins for the Ignition and Accessory relays as outputs.
    (PH1 and PK6, respectively) */
    MAP_GPIOPinTypeGPIOOutput(GPIO_PORTH_BASE, GPIO_PIN_1);
    MAP_GPIOPinTypeGPIOOutput(GPIO_PORTK_BASE, GPIO_PIN_6);

    /* Then, set them to LOW */
    MAP_GPIOPinWrite(GPIO_PORTH_BASE, GPIO_PIN_1, ~GPIO_PIN_1);
    MAP_GPIOPinWrite(GPIO_PORTK_BASE, GPIO_PIN_6, ~GPIO_PIN_6);

    /* Enable the GPIO pins for Ignition and Accessory Rx as inputs.
    (PP3 and PP5, respectively) */
    MAP_GPIOPinTypeGPIOInput(GPIO_PORTP_BASE, (GPIO_PIN_3 | GPIO_PIN_5));

    /* Then, enable the MCU's pull-down resistors on each. */
    GPIOP->PDR |= (GPIO_PIN_3 | GPIO_PIN_5);
}

uint32_t ignitionPoll(void)
{
    /* Poll if ignition Rx is HIGH. If so, output HIGH to ignition relay.
    Else, keep output to ignition relay LOW.
    Return state of ignition Rx as bit packed byte */


    uint32_t input = MAP_GPIOPinRead(GPIO_PORTP_BASE, GPIO_PIN_3);

    // Ignition switch, Rx: PP3, Relay Output: PH1
    if ( input == GPIO_PIN_3) {
        MAP_GPIOPinWrite(GPIO_PORTH_BASE, GPIO_PIN_1, GPIO_PIN_1);
    } else {
        MAP_GPIOPinWrite(GPIO_PORTH_BASE, GPIO_PIN_1, ~(GPIO_PIN_1));
    }

    return input;

}



uint32_t accessoryPoll(void)
{
    /* Poll if accessory Rx is HIGH. If so, output HIGH to accessory relay.
    Else, keep output to accessory relay LOW.
    Return state of accessory Rx as bit packed byte */

    uint32_t input = MAP_GPIOPinRead(GPIO_PORTP_BASE, GPIO_PIN_5);

    // Accessory switch, Rx: PP5, Relay Output: PK6
    if (input == GPIO_PIN_5) {
        MAP_GPIOPinWrite(GPIO_PORTK_BASE, GPIO_PIN_6, GPIO_PIN_6);
    } else {
        MAP_GPIOPinWrite(GPIO_PORTK_BASE, GPIO_PIN_6, ~(GPIO_PIN_6));
    }

    return input;

}


states prePump (void)
{
    /* Subroutine for PREPUMP state.
    Checks if accessory switch is closed.
    Changes relay output if so and returns next state.*/

    uint32_t accessStatus = MAP_GPIOPinRead(GPIO_PORTP_BASE, GPIO_PIN_5);
    while(accessStatus != GPIO_PIN_5)
    {
       accessStatus = AccessPoll();
    }

    return PREIGNIT;
}


states preIgnit (void)
{
    /* Subroutine for PREIGNIT state.
    Checks if accessory switch is opened or ignition switch is close.
    Changes relay output in response to one of the events and returns next state */


    uint32_t accessStatus = MAP_GPIOPinRead(GPIO_PORTP_BASE, GPIO_PIN_5);
    uint32_t ignitStatus = MAP_GPIOPinRead(GPIO_PORTP_BASE, GPIO_PIN_3);

    // while ACCESS ON, IGNIT OFF
    while(accessStatus == GPIO_PIN_5 && ignitStatus == ~GPIO_PIN_3)
    {
        accessStatus = accessoryPoll();
        ignitStatus = ignitionPoll();
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


states postIgnit (void)
{
    /* Subroutine for PREPUMP state.
    Checks if accessory switch is closed.
    Changes relay output if so and returns next state. */


    while(1)
    {

    }

    return;
}



void auxADCSetup()
{
    /* AUX ADC SETUP - built using adc0_singleended_singlechannel_singleseq */

    /* Enable the clock to GPIO Port E and wait for it to be ready */
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    while(!(MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOE))) {};

    /* Configure PE0 as ADC input channel */
    MAP_GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_0);

    /* Enable the clock to ADC0 and wait for it to be ready */
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
    while(!(MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_ADC0))) {};

    /* Configure Sequencer 3 to sample a single analog channel: AIN3 */
    MAP_ADCSequenceStepConfigure(ADC0_BASE, 3, 0, ADC_CTL_CH3 | ADC_CTL_IE | ADC_CTL_END);

    /* Configure and enable sample sequence 3 with a processor signal trigger */
    MAP_ADCSequenceConfigure(ADC0_BASE, 3, ADC_TRIGGER_PROCESSOR, 0);
    MAP_ADCSequenceEnable(ADC0_BASE, 3);

    /* Clear interrupt status flag */
    MAP_ADCIntClear(ADC0_BASE, 3);
}

uint32_t auxADCSend(uint32_t* auxBatVoltage)
{
    /* AUX ADC */
    MAP_ADCProcessorTrigger(ADC0_BASE, 3);
    while(!MAP_ADCIntStatus(ADC0_BASE, 3, false)) {}
    MAP_ADCIntClear(ADC0_BASE, 3);
    MAP_ADCSequenceDataGet(ADC0_BASE, 3, auxBatVoltage);

    uint8_t asciiChars[5];
    float tempFloat = auxBatVoltage[0];

    // From ((v/1000)/1.265)/.1904 - see spreadsheet
    tempFloat *= 0.004152;
    uint32_t temp = tempFloat * 100;
    uint32_t toReturn = temp;

    asciiChars[4] = temp % 10 + '0';
    temp /= 10;
    asciiChars[3] = temp % 10 + '0';
    temp /= 10;
    asciiChars[2] = '.';
    asciiChars[1] = temp % 10 + '0';
    temp /= 10;
    asciiChars[0] = temp % 10 + '0';

    UARTSend((uint8_t *)"AUX:", 4);
    UARTSend(asciiChars, 5);
    UARTSend((uint8_t *)"\n", 1);

    return toReturn;
}

void UART7Setup()
{
    /* UART Transmit Setup */

    /* Enable clock to peripherals used */
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART7);
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);

    /* Set PC4 and PC5 as UART pins */
    GPIOPinConfigure(GPIO_PC4_U7RX);
    GPIOPinConfigure(GPIO_PC5_U7TX);
    MAP_GPIOPinTypeUART(GPIO_PORTC_BASE, GPIO_PIN_4 | GPIO_PIN_5);

    /* Configure UART for 57,600, 8-N-1 */
    MAP_UARTConfigSetExpClk(UART7_BASE, systemClock, 57600,
                            UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE);
}
