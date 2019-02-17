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

// State names
typedef enum states { ACC, IGNIT, MAX_STATES } states;

// Function prototypes
void UARTSend(const uint8_t *pui8Buffer, uint32_t ui32Count);
void auxADCSetup();
uint32_t auxADCSend(uint32_t* auxBatVoltage);
void UART7Setup();
void switchesSetup(void);
uint32_t ignitPoll(void);
uint32_t accPoll(void);

// State Subroutines
states acc (void);
states ignit (void);

// Lookup array for subroutine of a state
states (*const state_array [MAX_STATES]) (void) = { acc, ignit };

int main(void)
{
    /* Configure system clock for 120 MHz */
    systemClock = MAP_SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ | SYSCTL_OSC_MAIN |
                                          SYSCTL_USE_PLL | SYSCTL_CFG_VCO_480),
                                          120000000);

    uint32_t auxBatVoltage[1];
    uint32_t auxBatAdjusted; //no decimal, accurate value
    states cur_state = ACC;

    auxADCSetup();
    UART7Setup();
    switchesSetup();


    // Loop forever.
    while (1)
    {

        auxBatAdjusted = auxADCSend(auxBatVoltage);

        if(cur_state < MAX_STATES) {
            cur_state = state_array[cur_state]();
        }

        MAP_SysCtlDelay(systemClock/2);
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
    /* Enables pins for ACC Tx/Rx, IGN Tx/Rx and the ACC/IGN relays */


    /* Enable clock peripherals used
    (H, K, M, P, Q) */
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOH);
    while(!MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOH)){};
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOK);
    while(!MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOK)){};
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOM);
    while(!MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOM)){};
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOP);
    while(!MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOP)){};
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOQ);
    while(!MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOQ)){};


    /*Enable the GPIO Pins for Ignition and Accessory Tx as outputs.
    ( PM6 and PQ1, respectively) */
    MAP_GPIOPinTypeGPIOOutput(GPIO_PORTM_BASE, GPIO_PIN_6);
    MAP_GPIOPinTypeGPIOOutput(GPIO_PORTQ_BASE, GPIO_PIN_1);

    /* Set them to HIGH */
    MAP_GPIOPinWrite(GPIO_PORTM_BASE, GPIO_PIN_6, GPIO_PIN_6);
    MAP_GPIOPinWrite(GPIO_PORTQ_BASE, GPIO_PIN_1, GPIO_PIN_1);

    /* Enable the GPIO Pins for the Ignition and Accessory relays as outputs.
    (PH0 and PK4, respectively) */
    MAP_GPIOPinTypeGPIOOutput(GPIO_PORTH_BASE, GPIO_PIN_0);
    MAP_GPIOPinTypeGPIOOutput(GPIO_PORTK_BASE, GPIO_PIN_4);

    /* Then, set them to LOW */
    MAP_GPIOPinWrite(GPIO_PORTH_BASE, GPIO_PIN_0, ~GPIO_PIN_0);
    MAP_GPIOPinWrite(GPIO_PORTK_BASE, GPIO_PIN_4, ~GPIO_PIN_4);

    /* Enable the GPIO pins for Ignition and Accessory Rx as inputs.
    (PP3 and PH1, respectively) */
    MAP_GPIOPinTypeGPIOInput(GPIO_PORTP_BASE, GPIO_PIN_3);
    MAP_GPIOPinTypeGPIOInput(GPIO_PORTH_BASE, GPIO_PIN_1);

    /* Then, enable the MCU's pull-down resistors on each to prevent noise */
    GPIOP->PDR |= GPIO_PIN_3;
    GPIOH->PDR |= GPIO_PIN_1;
}

uint32_t ignitPoll(void)
{
    /* Poll if ignition Rx is HIGH. If so, output HIGH to ignition relay.
    Else, keep output to ignition relay LOW.
    Return whether ignition Rx reads HIGH or LOW as bit packed byte */


    uint32_t input = MAP_GPIOPinRead(GPIO_PORTP_BASE, GPIO_PIN_3);

    // Ignition switch, Rx: PP3, Relay Output: PH0
    if ( input == GPIO_PIN_3) {
        MAP_GPIOPinWrite(GPIO_PORTH_BASE, GPIO_PIN_0, GPIO_PIN_0);
    } else {
        MAP_GPIOPinWrite(GPIO_PORTH_BASE, GPIO_PIN_0, ~(GPIO_PIN_0));
    }

    return input;

}



uint32_t accPoll(void)
{
    /* Poll if accessory Rx is HIGH. If so, output HIGH to accessory relay.
    Else, keep output to accessory relay LOW.
    Return whether accessory Rx reads HIGH or LOW as bit packed byte */

    uint32_t input = MAP_GPIOPinRead(GPIO_PORTH_BASE, GPIO_PIN_1);

    // Accessory switch, Rx: PH1, Relay Output: PK4
    if (input == GPIO_PIN_1) {
        MAP_GPIOPinWrite(GPIO_PORTK_BASE, GPIO_PIN_4, GPIO_PIN_4);
    } else {
        MAP_GPIOPinWrite(GPIO_PORTK_BASE, GPIO_PIN_4, ~(GPIO_PIN_4));
    }

    return input;

}


states acc (void)
{
    /* Subroutine for ACC state */

    uint32_t accStatus = MAP_GPIOPinRead(GPIO_PORTP_BASE, GPIO_PIN_5);
    while(accStatus != GPIO_PIN_5)
    {
       accStatus = accPoll();
    }

    return IGNIT;
}


states ignit (void)
{
    /* Subroutine for IGNIT state. */

    uint32_t accStatus = MAP_GPIOPinRead(GPIO_PORTP_BASE, GPIO_PIN_5);
    uint32_t ignitStatus = MAP_GPIOPinRead(GPIO_PORTP_BASE, GPIO_PIN_3);

    // while ACCESS ON, IGNIT OFF
    while(accStatus == GPIO_PIN_5 && ignitStatus == ~GPIO_PIN_3)
    {
        accStatus = accPoll();
        ignitStatus = ignitPoll();
    }

    // Decides which state to return
    // One of 4 combinations of access Rx and ignit Rx
    // GPIO_PIN_5 is  00001000 and GPIO_PIN_3 00000010
    switch (accStatus | ignitStatus )
    {
        // ACC RELAY ON, IGNIT RELAY ON
        case (GPIO_PIN_5 | GPIO_PIN_3):
            return IGNIT;
            break;

        // ACC RELAY OFF, IGNIT RELAY OFF
        case (~GPIO_PIN_5 | ~GPIO_PIN_3):
            return ACC;
            break;

        // ACC RELAY OFF, IGNIT RELAY ON
        default:
            return ACC;
            break;
    }
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
