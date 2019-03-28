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
#include "UARTstdio.h"

/* Standard driverlib include - can be more specific if needed */
#include <ti/devices/msp432e4/driverlib/driverlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

// ID of each CAN message
// Byte the message starts at (out of 8)
// ASCII length of data (1 data bytes -> 3 ASCII bytes, 2 data bytes -> 5 ASCII bytes)
#define SOC_ID           0x6B0
#define SOC_byte         4
#define SOC_length       3

#define FPV_ID           0x6B0
#define FPV_byte         2
#define FPV_length       5

#define highTempID       0x6B4
#define highTempByte     0
#define lowTempID        0x6B4
#define lowTempByte      2
#define tempLength       5

#define highVoltageID    0x6B3
#define highVoltageByte  2
#define lowVoltageID     0x6B3
#define lowVoltageByte   4
#define voltageLength    5

#define UART_2_EN

/* Configure system clock for 120 MHz */
uint32_t systemClock;

/* CAN variables */
bool rxMsg = false;
bool errFlag = false;
uint32_t msgCount = 0;

// CAN data struct
typedef struct {                         // Multiplication factors (units) from the BMS utility manual
    uint8_t SOC[SOC_length];             // 0.5 (%)
    uint8_t FPV[FPV_length];             // 0.1 (V)
    uint8_t highTemp[tempLength];        // 1 (degrees C) (BOLT3 data indicates 0.1)
    uint8_t lowTemp[tempLength];         // 1 (degrees C) (BOLT3 data indicates 0.1)
    uint8_t highVoltage[voltageLength];  // 0.0001 (V)
    uint8_t lowVoltage[voltageLength];   // 0.0001 (V)
} CANTransmitData;

// Function prototypes
void UARTSend(const uint8_t *pui8Buffer, uint32_t ui32Count);
void auxADCSetup();
uint32_t auxADCSend(uint32_t* auxBatVoltage);
void UART7Setup();
void enableUARTprintf();
void switchesSetup(void);
void ignitPoll(void);
void accPoll(void);
void canSetup(tCANMsgObject* message);
void configureCAN();
void canReceive(tCANMsgObject* sCANMessage, CANTransmitData* CANdata,
                uint8_t msgDataIndex, uint8_t* msgData);
void convertToASCII(uint8_t* chars, uint8_t digits, uint16_t num);
void enableLEDs();
void toggleLED1();
void toggleLED2();
void toggleLED3();
void toggleLED4();



int main(void)
{
    /* Configure system clock for 120 MHz */
    systemClock = MAP_SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ | SYSCTL_OSC_MAIN |
                                          SYSCTL_USE_PLL | SYSCTL_CFG_VCO_480),
                                          120000000);

    tCANMsgObject sCANMessage;
    uint8_t msgDataIndex;
    uint8_t msgData[8] = {0x00, 0x00, 0x00, 0x00,
                          0x00, 0x00, 0x00, 0x00};

    uint32_t auxBatVoltage[1];
    uint32_t auxBatAdjusted; //no decimal, accurate value


    auxADCSetup();
    UART7Setup();
    switchesSetup();
    configureCAN();
    canSetup(&sCANMessage);

    enableUARTprintf();
    UARTprintf("Starting up\n");

    enableLEDs();

    // Loop forever.
    while (1)
    {
        static CANTransmitData CANData;
        //accPoll();
        //ignitPoll();
        canReceive(&sCANMessage, &CANData, msgDataIndex, msgData);
    }
}

void enableUARTprintf()
{
    /* Enable the GPIO Peripheral used by the UART */
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    while(!(MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOD))) {}

    /* Enable UART2 */
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART2);

    /* Configure GPIO Pins for UART mode */
    MAP_GPIOPinConfigure(GPIO_PD4_U2RX);
    MAP_GPIOPinConfigure(GPIO_PD5_U2TX);
    MAP_GPIOPinTypeUART(GPIO_PORTD_BASE, GPIO_PIN_4 | GPIO_PIN_5);

    /* Initialize the UART for console I/O */
    UARTStdioConfig(2, 115200, systemClock);
}

void canSetup(tCANMsgObject* message)
{
    /* Enable the clock to the GPIO Port J and wait for it to be ready */
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    while(!(MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOA)))
    {
    }

    /* Initialize the CAN */
    configureCAN();

    /* Initialize message object 1 to be able to receive any CAN message ID.
     * In order to receive any CAN ID, the ID and mask must both be set to 0,
     * and the ID filter enabled */
    message->ui32MsgID = 0;
    message->ui32MsgIDMask = 0;

    /* Enable interrupt on RX and Filter ID */
    message->ui32Flags = MSG_OBJ_RX_INT_ENABLE | MSG_OBJ_USE_ID_FILTER;

    /* Size of message is 8 */
    message->ui32MsgLen = 8; //could also be sizeof(msgData)

    /* Load the message object into the CAN peripheral. Once loaded an
     * interrupt will occur any time a CAN message is received. Use message
     * object 1 for receiving messages */
    MAP_CANMessageSet(CAN0_BASE, 1, message, MSG_OBJ_TYPE_RX);
}

void configureCAN(void)
{
    /* Configure the CAN and its pins PA0 and PA1 @ 500Kbps */

    /* Enable the clock to the GPIO Port A and wait for it to be ready */
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    while(!(MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOA)))
    {
    }

    /* Enable CAN0 */
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_CAN0);

    /* Configure GPIO Pins for CAN mode */
    MAP_GPIOPinConfigure(GPIO_PA0_CAN0RX);
    MAP_GPIOPinConfigure(GPIO_PA1_CAN0TX);
    MAP_GPIOPinTypeCAN(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    /* Initialize the CAN controller */
    MAP_CANInit(CAN0_BASE);

    /* Set up the bit rate for the CAN bus.  CAN bus is set to 500 Kbps */
    MAP_CANBitRateSet(CAN0_BASE, systemClock, 500000);

    /* Enable interrupts on the CAN peripheral */
    MAP_CANIntEnable(CAN0_BASE, CAN_INT_MASTER | CAN_INT_ERROR | CAN_INT_STATUS);

    /* Enable the CAN interrupt */
    MAP_IntEnable(INT_CAN0);

    /* Enable the CAN for operation */
    MAP_CANEnable(CAN0_BASE);
}

void canReceive(tCANMsgObject* sCANMessage, CANTransmitData* CANData, uint8_t msgDataIndex, uint8_t* msgData)
{
    uint8_t cycleMsgs = 0;
    /* A new message is received */
    while (cycleMsgs <= 6)
    {
        if (rxMsg)
        {
            /* Re-use the same message object that was used earlier to configure
             * the CAN */
            sCANMessage->pui8MsgData = (uint8_t *)msgData;

            /* Read the message from the CAN */
            MAP_CANMessageGet(CAN0_BASE, 1, sCANMessage, 0);

            // Set rxMsg to false since data has been put into the object
            // The next message will stay in the buffer until CANMessageGet() is called again
            // The interrupt function sets rxMsg to true to satisfy the if statement
            rxMsg = false;

            /* Check the error flag to see if errors occurred */
            if (sCANMessage->ui32Flags & MSG_OBJ_DATA_LOST)
            {
                  UARTprintf("\nCAN message loss detected\n");
            }
            else
            {
                /* Print a message to the console showing the message count and the
                 * contents of the received message */
                UARTprintf("Message length: %i \n", sCANMessage->ui32MsgLen);
                UARTprintf("Received msg 0x%03X: ",sCANMessage->ui32MsgID);
                for (msgDataIndex = 0; msgDataIndex < sCANMessage->ui32MsgLen;
                        msgDataIndex++)
                {
                    UARTprintf("0x%02X ", msgData[msgDataIndex]);
                }

                /* Print the count of message sent */
                UARTprintf(" total count = %u\n", msgCount);

                static uint8_t SOCtemp = 0;
                static uint16_t FPVtemp = 0;
                static uint16_t highTempTemp = 0;
                static uint16_t lowTempTemp = 0;
                static uint16_t highVoltTemp = 0;
                static uint16_t lowVoltTemp = 0;

                // Populates the temporary variables
                if (sCANMessage->ui32MsgID == SOC_ID) {
                    SOCtemp = msgData[SOC_byte];
                }
                if (sCANMessage->ui32MsgID == FPV_ID) {
                    FPVtemp = (msgData[FPV_byte+1] << 8) | msgData[FPV_byte];
                }
                if (sCANMessage->ui32MsgID == highTempID) {
                    highTempTemp = (msgData[highTempByte+1] << 8) | msgData[highTempByte];
                }
                if (sCANMessage->ui32MsgID == lowTempID) {
                    lowTempTemp = (msgData[lowTempByte+1] << 8) | msgData[lowTempByte];
                }
                if (sCANMessage->ui32MsgID == highVoltageID) {
                    highVoltTemp = (msgData[highVoltageByte+1] << 8) | msgData[highVoltageByte];
                }
                if (sCANMessage->ui32MsgID == lowVoltageID) {
                    lowVoltTemp = (msgData[lowVoltageByte+1] << 8) | msgData[lowVoltageByte];
                }

                // Processes numbers into ASCII
                convertToASCII(CANData->SOC, 3, SOCtemp);
                convertToASCII(CANData->FPV, 5, FPVtemp);
                convertToASCII(CANData->highTemp, 5, highTempTemp);
                convertToASCII(CANData->lowTemp, 5, lowTempTemp);
                convertToASCII(CANData->lowVoltage, 5, lowVoltTemp);
                convertToASCII(CANData->highVoltage, 5, highVoltTemp);

                // Print to UART console

                UARTprintf("SOC: %c%c%c\n", CANData->SOC[0], CANData->SOC[1], CANData->SOC[2]);
                UARTprintf("FPV: %c%c%c%c%c\n", CANData->FPV[0], CANData->FPV[1], CANData->FPV[2], CANData->FPV[3], CANData->FPV[4]);
                UARTprintf("highTemp: %c%c%c%c%c\n", CANData->highTemp[0], CANData->highTemp[1], CANData->highTemp[2], CANData->highTemp[3], CANData->highTemp[4]);
                UARTprintf("lowTemp: %c%c%c%c%c\n", CANData->lowTemp[0], CANData->lowTemp[1], CANData->lowTemp[2], CANData->lowTemp[3], CANData->lowTemp[4]);
                UARTprintf("highVoltage: %c%c%c%c%c\n", CANData->highVoltage[0], CANData->highVoltage[1], CANData->highVoltage[2], CANData->highVoltage[3], CANData->highVoltage[4]);
                UARTprintf("lowVoltage: %c%c%c%c%c\n", CANData->lowVoltage[0], CANData->lowVoltage[1], CANData->lowVoltage[2], CANData->lowVoltage[3], CANData->lowVoltage[4]);

            }
        }
        else
        {
            if(errFlag)
            {
                UARTprintf("Error: Problem while receiving CAN interrupt");
            }
        }
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

void accPoll(void)
{
    /* Poll if accessory Rx is HIGH. If so, output HIGH to accessory relay.
    Else, keep output to accessory relay LOW.
    Return whether accessory Rx reads HIGH or LOW as bit packed byte */

    // bool input = MAP_GPIOPinRead(GPIO_PORTH_BASE, GPIO_PIN_1);

    // Accessory switch, Rx: PH1, Relay Output: PK4
    if (MAP_GPIOPinRead(GPIO_PORTH_BASE, GPIO_PIN_1) == GPIO_PIN_1) {
        MAP_GPIOPinWrite(GPIO_PORTK_BASE, GPIO_PIN_4, GPIO_PIN_4);
    } else {
        MAP_GPIOPinWrite(GPIO_PORTK_BASE, GPIO_PIN_4, ~(GPIO_PIN_4));
    }

    // return input;

}

void ignitPoll(void)
{
    /* Poll if ignition Rx is HIGH. If so, output HIGH to ignition relay.
    Else, keep output to ignition relay LOW.
    Return whether ignition Rx reads HIGH or LOW as bit packed byte */


    // bool input = MAP_GPIOPinRead(GPIO_PORTP_BASE, GPIO_PIN_3);

    // Ignition switch, Rx: PP3, Relay Output: PH0
    if (  MAP_GPIOPinRead(GPIO_PORTP_BASE, GPIO_PIN_3) == GPIO_PIN_3) {
        MAP_GPIOPinWrite(GPIO_PORTH_BASE, GPIO_PIN_0, GPIO_PIN_0);
    } else {
        MAP_GPIOPinWrite(GPIO_PORTH_BASE, GPIO_PIN_0, ~(GPIO_PIN_0));
    }

    // return input;

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
        MAP_UARTCharPut(UART7_BASE, *pui8Buffer++);
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

void convertToASCII(uint8_t* chars, uint8_t digits, uint16_t num)
{
    for ( ; digits > 0; digits--)
    {
        chars[digits-1] = num % 10 + '0';
        num /= 10;
    }
}

void CAN0_IRQHandler(void) // Uses CAN0, on J5
{
    uint32_t canStatus;

    /* Read the CAN interrupt status to find the cause of the interrupt */
    canStatus = MAP_CANIntStatus(CAN0_BASE, CAN_INT_STS_CAUSE);

    /* If the cause is a controller status interrupt, then get the status */
    if(canStatus == CAN_INT_INTID_STATUS)
    {
        /* Read the controller status.  This will return a field of status
         * error bits that can indicate various errors */
        canStatus = MAP_CANStatusGet(CAN0_BASE, CAN_STS_CONTROL);

        /* Set a flag to indicate some errors may have occurred */
        errFlag = true;
    }

    /* Check if the cause is message object 1, which what we are using for
     * receiving messages */
    else if(canStatus == 1)
    {
        /* Getting to this point means that the RX interrupt occurred on
         * message object 1, and the message RX is complete.  Clear the
         * message object interrupt */
        MAP_CANIntClear(CAN0_BASE, 1);

        /* Increment a counter to keep track of how many messages have been
         * sent. In a real application this could be used to set flags to
         * indicate when a message is sent */
        msgCount++;

        /* Set flag to indicate received message is pending */
        rxMsg = true;

        /* Since the message was sent, clear any error flags */
        errFlag = false;
    }
    else
    {
    }
}

void enableLEDs()
{
    /* Enable the GPIO port that is used for the on-board LED */
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION);
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);

    /* Check if the peripheral access is enabled */
    while(!MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_GPION))
    {
    }

    /* Check if the peripheral access is enabled */
    while(!MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF))
    {
    }

    /* Enable the GPIO pins for the LEDs (PN0 and PN1). Set the direction as output,
     * and enable the GPIO pin for digital function */
    MAP_GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_0 | GPIO_PIN_1,
                     ~(GPIO_PIN_0 | GPIO_PIN_1));
    MAP_GPIOPinTypeGPIOOutput(GPIO_PORTN_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    /* Enable the GPIO pins for the LEDs (PF0 and PF4). Set the direction as output,
     * and enable the GPIO pin for digital function */
    MAP_GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_0 | GPIO_PIN_4,
                     ~(GPIO_PIN_0 | GPIO_PIN_4));
    MAP_GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_0 | GPIO_PIN_4);
}

void toggleLED1()
{
    MAP_GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_1, ~(MAP_GPIOPinRead(GPIO_PORTN_BASE, GPIO_PIN_1)));
}

void toggleLED2()
{
    MAP_GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_0, ~(MAP_GPIOPinRead(GPIO_PORTN_BASE, GPIO_PIN_0)));
}

void toggleLED3()
{
    MAP_GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_4, ~(MAP_GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_4)));
}
void toggleLED4()
{
    MAP_GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_0, ~(MAP_GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_0)));
}
