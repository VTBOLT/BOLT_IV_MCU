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
#include "uartstdio.h"

uint32_t systemClock;

typedef enum states { PCB, ACC, IGN, MAX_STATES } states_t;
// Required second count to delay switching off ACC and IGN if voltage dips below for a brief second.
// We want to calculate the # of cycles to delay x seconds from the formula below:
// x seconds delay = (# of Cycles) * (1/frequency)
// So plug in the second count that you want delayed into the formula below:
// (120MHz)*(x second delay) = # of cycles
// The current value is for a 3 second delay: (120MHz)*(3s)=360000000
#define REQSECCOUNT 360000000

// Function prototypes
void UARTSend(const uint8_t *pui8Buffer, uint32_t ui32Count);
void auxADCSetup();
uint32_t auxADCSend(uint32_t* auxBatVoltage);
void UART7Setup();
void accIgnDESetup(void);
void timerSetup();
void timerRun();
bool ignitPoll(void);
bool accPoll(void);
bool DEPoll(void);


int main(void)
{
    /* Configure system clock for 120 MHz */
    systemClock = MAP_SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ | SYSCTL_OSC_MAIN |
                                          SYSCTL_USE_PLL | SYSCTL_CFG_VCO_480),
                                          120000000);
    // Enable interrupts globally
    MAP_IntMasterEnable();

    // Set up the timer system
    timerSetup();

    uint32_t auxBatVoltage[1];
    uint32_t auxBatAdjusted; //no decimal, accurate value

    // Enables UARTprintf
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    MAP_GPIOPinConfigure(GPIO_PA0_U0RX);
    MAP_GPIOPinConfigure(GPIO_PA1_U0TX);
    MAP_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    UARTStdioConfig(0, 115200, systemClock);

    while(!(MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_ADC0)))

    auxADCSetup();
    UART7Setup();
    accIgnDESetup();

    states_t present = PCB;

    // Loop forever.
    while (1)
    {
        //auxADCSend(auxBatVoltage);
        switch(present)
        {

        /* ACC state of FSM */
        case ACC:

            // Output HIGH to ACC Relay
            MAP_GPIOPinWrite(GPIO_PORTK_BASE, GPIO_PIN_4, GPIO_PIN_4);
            // Output HIGH to ACC Dash
            MAP_GPIOPinWrite(GPIO_PORTK_BASE, GPIO_PIN_6, GPIO_PIN_6);

            // Output LOW to IGN Relay
            MAP_GPIOPinWrite(GPIO_PORTH_BASE, GPIO_PIN_0, ~GPIO_PIN_0);
            // Output LOW to IGN Dash
            MAP_GPIOPinWrite(GPIO_PORTK_BASE, GPIO_PIN_7, ~GPIO_PIN_7);

            // Output HIGH to PSI LED
            MAP_GPIOPinWrite(GPIO_PORTP_BASE, GPIO_PIN_2, GPIO_PIN_2);
            // Output HIGH to PSI Dash
            MAP_GPIOPinWrite(GPIO_PORTM_BASE, GPIO_PIN_2, GPIO_PIN_2);


            // Aux battery voltage stored as volts * 1000
            auxBatAdjusted = auxADCSend(auxBatVoltage);

            if (auxBatAdjusted <= 1200) {

                //  Wait 3 seconds to check value, ensure constant value
                timerRun();

                auxBatAdjusted = auxADCSend(auxBatVoltage);
                if (auxBatAdjusted <= 1200) {
                    present = PCB;
                }

            } else if (!accPoll()) {

                present = PCB;

            } else if (ignitPoll()){

                present = IGN;
            }
            break;

        /* IGN state of FSM */
        case IGN:

            if (!DEPoll()) {

                // Output HIGH to ACC Relay
                MAP_GPIOPinWrite(GPIO_PORTK_BASE, GPIO_PIN_4, GPIO_PIN_4);
                // Output HIGH to ACC Dash
                MAP_GPIOPinWrite(GPIO_PORTK_BASE, GPIO_PIN_6, GPIO_PIN_6);

                // Output HIGH to IGN Relay
                MAP_GPIOPinWrite(GPIO_PORTH_BASE, GPIO_PIN_0, GPIO_PIN_0);
                // Output HIGH to IGN Dash
                MAP_GPIOPinWrite(GPIO_PORTK_BASE, GPIO_PIN_7, GPIO_PIN_7);

                // Output HIGH to PSI LED
                MAP_GPIOPinWrite(GPIO_PORTP_BASE, GPIO_PIN_2, GPIO_PIN_2);
                // Output HIGH to PSI LED
                MAP_GPIOPinWrite(GPIO_PORTM_BASE, GPIO_PIN_2, GPIO_PIN_2);


                auxBatAdjusted = auxADCSend(auxBatVoltage);

                if (auxBatAdjusted <= 1200) {

                //  Wait 3 seconds to check value, ensure constant value
                    timerRun();

                    auxBatAdjusted = auxADCSend(auxBatVoltage);
                    if (auxBatAdjusted <= 1200) {
                        present = ACC;
                    }

                //} else if (/*Low pump current*/) {

                    //present = ACC;

                } else if (!ignitPoll()) {

                    present = ACC;

                } else if (!accPoll()) {

                    present = ACC;

                }

            } else {

                present = ACC;
            }
            break;

        /* PCB state of FSM */
        default:

            // Output LOW to ACC Relay
            MAP_GPIOPinWrite(GPIO_PORTK_BASE, GPIO_PIN_4, ~GPIO_PIN_4);
            // Output LOW to ACC Dash
            MAP_GPIOPinWrite(GPIO_PORTK_BASE, GPIO_PIN_6, ~GPIO_PIN_6);

            // Output LOW to IGN Relay
            MAP_GPIOPinWrite(GPIO_PORTH_BASE, GPIO_PIN_0, ~GPIO_PIN_0);
            // Output LOW to IGN Dash
            MAP_GPIOPinWrite(GPIO_PORTK_BASE, GPIO_PIN_7, ~GPIO_PIN_7);

            // Output LOW to PSI LED
            MAP_GPIOPinWrite(GPIO_PORTP_BASE, GPIO_PIN_2, ~GPIO_PIN_2);
            //Output LOW to PSI Dash
            MAP_GPIOPinWrite(GPIO_PORTM_BASE, GPIO_PIN_2, ~GPIO_PIN_2);


            if (accPoll()) {
                present = ACC;
            }
            break;

        }
    }
}

void accIgnDESetup(void)
{
    // ACC K6, IGN K7, PSI M2
    /* Enables pins for ACC Tx/Rx, IGN Tx/Rx, ACC/IGN relays and reading from BMS Discharge Enable  */


    /* Enable clock to peripherals used (H, K, M, P, Q) */
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
    MAP_GPIOPinTypeGPIOOutput(GPIO_PORTP_BASE, GPIO_PIN_2);

    /* Then, set them to LOW */
    MAP_GPIOPinWrite(GPIO_PORTH_BASE, GPIO_PIN_0, ~GPIO_PIN_0);
    MAP_GPIOPinWrite(GPIO_PORTK_BASE, GPIO_PIN_4, ~GPIO_PIN_4);

    /* Enable the GPIO Pins for the Accessory, Ignition, and PSI Dash outputs.
    (PK6, PK7, PM2, respectively) */
    MAP_GPIOPinTypeGPIOOutput(GPIO_PORTK_BASE, GPIO_PIN_6);
    MAP_GPIOPinTypeGPIOOutput(GPIO_PORTK_BASE, GPIO_PIN_7);
    MAP_GPIOPinTypeGPIOOutput(GPIO_PORTM_BASE, GPIO_PIN_2);

    /* Then set them to LOW */
    MAP_GPIOPinWrite(GPIO_PORTK_BASE, GPIO_PIN_6, ~GPIO_PIN_6);
    MAP_GPIOPinWrite(GPIO_PORTK_BASE, GPIO_PIN_7, ~GPIO_PIN_7);
    MAP_GPIOPinWrite(GPIO_PORTM_BASE, GPIO_PIN_2, ~GPIO_PIN_2);

    /* Enable the GPIO pins for Ignition and Accessory Rx as inputs.
    (PP3 and PH1, respectively) */
    MAP_GPIOPinTypeGPIOInput(GPIO_PORTP_BASE, GPIO_PIN_3);
    MAP_GPIOPinTypeGPIOInput(GPIO_PORTH_BASE, GPIO_PIN_1);

    /* Enable the GPIO pin for BMS Discharge Enable as input.
    (PP2) */
    MAP_GPIOPinTypeGPIOInput(GPIO_PORTM_BASE, GPIO_PIN_1);

    /* Then, enable the MCU's pull-down resistors on each to prevent noise */
    GPIOP->PDR |= GPIO_PIN_3;
    GPIOH->PDR |= GPIO_PIN_1;

    /* Enable MCU's pull-up resistor on PM1 for reading from BMS Discharge Enable */
    GPIOM->PUR |= GPIO_PIN_1;
}

bool accPoll(void)
{
    /* Return TRUE if acc Rx reads HIGH and FALSE if acc Rx reads LOW.
       As a note: ACC Rx: PH1, ACC Relay: PK4 */

    if (MAP_GPIOPinRead(GPIO_PORTH_BASE, GPIO_PIN_1) == GPIO_PIN_1) {
        return true;
    } else {
        return false;
    }

}

bool ignitPoll(void)
{
    /* Return TRUE if ignition Rx reads HIGH and FALSE if ignition Rx reads LOW
       As a note: IGN Rx: PP3, IGN Relay: PH0 */

    if (  MAP_GPIOPinRead(GPIO_PORTP_BASE, GPIO_PIN_3) == GPIO_PIN_3) {
        return true;
    } else {
        return false;
    }

}

bool DEPoll(void)
{
    /* Poll if the bike is able to be discharged.
       Return TRUE if so, return FALSE otherwise. */

    /* DE is read from PM1. Pull-up resistor is enabled on PM1 since
       the BMS grounds PM1 when the bike can discharge. */
    if (MAP_GPIOPinRead(GPIO_PORTM_BASE, GPIO_PIN_1) == GPIO_PIN_1) {
        return false;
    } else {
        return true;
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
    /* BUG
     * @ 12 V in, this function outputs 1120 (11.2V)
     * This is safe so long as we're underestimating
     * Consider for future revisions
     */

    /* AUX ADC */
    MAP_ADCProcessorTrigger(ADC0_BASE, 3);
    while(!MAP_ADCIntStatus(ADC0_BASE, 3, false)) {}
    MAP_ADCIntClear(ADC0_BASE, 3);
    MAP_ADCSequenceDataGet(ADC0_BASE, 3, auxBatVoltage);

    uint8_t asciiChars[5];
    float tempFloat = auxBatVoltage[0];

    // From ((v/1000)/1.265)/.1904 - see spreadsheet
    tempFloat *= 0.004719;
    uint32_t temp = tempFloat * 100;
    uint32_t toReturn = temp;

    UARTprintf("Zero: %d\n", auxBatVoltage[0]);
    UARTprintf("AUX: %d\n", temp);

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

void timerSetup() {
    // Set the 32-bit timer Peripheral.
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
    // Configure the timer to be one-shot.
    MAP_TimerConfigure(TIMER0_BASE, TIMER_CFG_ONE_SHOT);
}

void timerRun() {
    // Load the required second count into the timer.
    MAP_TimerLoadSet(TIMER0_BASE, TIMER_A, REQSECCOUNT);

    // Enable the timer.
    MAP_TimerEnable(TIMER0_BASE, TIMER_A);

    // Wait for 3 seconds to have the timer delay the system
    while(MAP_TimerValueGet(TIMER0_BASE, TIMER_A) != REQSECCOUNT) {}
}
