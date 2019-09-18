#include "msp.h"

/* Standard driverlib include - can be more specific if needed */
#include <ti/devices/msp432e4/driverlib/driverlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "uartstdio.h"

#include "can_comms.h"
#include "uart_comms.h"

/* CAN variables */
bool rxMsg = false;
bool errFlag = false;
uint32_t msgCount = 0;

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

    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOL);
    while(!MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOL)){};

    MAP_GPIOPinTypeGPIOOutput(GPIO_PORTL_BASE, GPIO_PIN_3);
    MAP_GPIOPinWrite(GPIO_PORTL_BASE, GPIO_PIN_3, ~(GPIO_PIN_3));

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

void canReceive(tCANMsgObject* sCANMessage, CANTransmitData_t* CANData, uint8_t msgDataIndex, uint8_t* msgData)
{
    uint8_t cycleMsgs = 0;
    /* A new message is received */
    while (cycleMsgs <= 6)
    {
        cycleMsgs++;
        //UARTprintf("In canReceive loop, cycleMsgs = %d\n", cycleMsgs);
        if (rxMsg)
        {
            /* Re-use the same message object that was used earlier to configure
             * the CAN */
            sCANMessage->pui8MsgData = (uint8_t *)msgData;

            /* Read the message from the CAN */
            MAP_CANMessageGet(CAN0_BASE, 1, sCANMessage, 0);

            // Set rxMsg to false since data has been put into the object
            // The next message will stay in the buffer until CANMessageGet() is called again
            //The interrupt functi.on sets rxMsg to true to satisfy the if statement
            rxMsg = false;

            /* Print a message to the console showing the message count and the
             * contents of the received message
            UARTprintf("Message length: %i \n", sCANMessage->ui32MsgLen);
            UARTprintf("Received msg 0x%03X: ",sCANMessage->ui32MsgID);
            for (msgDataIndex = 0; msgDataIndex < sCANMessage->ui32MsgLen;
                    msgDataIndex++)
            {
                UARTprintf("0x%02X ", msgData[msgDataIndex]);
            }

            /* Print the count of message sent
            UARTprintf(" total count = %u\n", msgCount);*/

            static uint8_t SOCtemp = 20;
            static uint16_t FPVtemp = 21;
            static uint16_t highTempTemp = 10;
            static uint16_t lowTempTemp = 10;
            static uint16_t highVoltTemp = 24;
            static uint16_t lowVoltTemp = 25;
            static int16_t rpmTemp = 26;
            static int16_t motorTempTemp = 27;
            static int16_t motorCtrlTempTemp = 28;
            static uint16_t dcBusCurrentTemp = 29;
            static uint16_t motorTorqueTemp = 30;

            // Populates the temporary variables
            if (sCANMessage->ui32MsgID == highTempID) {
                highTempTemp = (msgData[highTempByte+1] << 8) | msgData[highTempByte];
            }
            if (sCANMessage->ui32MsgID == lowTempID) {
                lowTempTemp = (msgData[lowTempByte+1] << 8) | msgData[lowTempByte];
            }
            if (sCANMessage->ui32MsgID == SOC_ID) {
                SOCtemp = msgData[SOC_byte];
            }
            if (sCANMessage->ui32MsgID == FPV_ID) {
                FPVtemp = (msgData[FPV_byte+1] << 8) | msgData[FPV_byte];
            }
            if (sCANMessage->ui32MsgID == highVoltageID) {
                highVoltTemp = (msgData[highVoltageByte+1] << 8) | msgData[highVoltageByte];
            }
            if (sCANMessage->ui32MsgID == lowVoltageID) {
                lowVoltTemp = (msgData[lowVoltageByte+1] << 8) | msgData[lowVoltageByte];
            }
            if (sCANMessage->ui32MsgID == RPM_ID) {
                rpmTemp = (msgData[RPM_BYTE+1] << 8) | msgData[RPM_BYTE];
            }
            if (sCANMessage->ui32MsgID == motorTempID) {
                motorTempTemp = (msgData[motorTempByte+1] << 8) | msgData[motorTempByte];
            }
            if (sCANMessage->ui32MsgID == motorCtrlTempID) {
                motorCtrlTempTemp = (msgData[motorCtrlTempByte+1] << 8) | msgData[motorCtrlTempByte];
            }
            if (sCANMessage->ui32MsgID == motorTorqueID) {
                motorTorqueTemp = (msgData[motorTorqueByte+1] << 8) | msgData[motorTorqueByte];
            }
            if (sCANMessage->ui32MsgID == dcBusCurrentID) {
                dcBusCurrentTemp = (msgData[dcBusCurrentByte+1] << 8) | msgData[dcBusCurrentByte];
            }

            /* Set ID to 0 so the next message can be received
             * May be unnecessary */
            sCANMessage->ui32MsgID = 0;

            // Processes numbers into ASCII
            convertToASCII(CANData->SOC, 3, SOCtemp);
            convertToASCII(CANData->FPV, 5, FPVtemp);
            convertToASCII(CANData->highTemp, 5, highTempTemp);
            convertToASCII(CANData->lowTemp, 5, lowTempTemp);
            convertToASCII(CANData->lowVoltage, 5, lowVoltTemp);
            convertToASCII(CANData->highVoltage, 5, highVoltTemp);
            convertToASCII(CANData->RPM, RPM_LEN, rpmTemp);
            convertToASCII(CANData->motorTemp, motorTempLength, motorTempTemp);
            convertToASCII(CANData->motorCtrlTemp, motorCtrlTempLen, motorCtrlTempTemp);
            convertToASCII(CANData->motorTorque, motorTorqueLen, motorTorqueTemp);
            convertToASCII(CANData->dcBusCurrent, dcBusCurrentLen, dcBusCurrentTemp);

            // Print to UART console
/*
            UARTprintf("SOC: %c%c%c\n", CANData->SOC[0], CANData->SOC[1], CANData->SOC[2]);
            UARTprintf("FPV: %c%c%c%c%c\n", CANData->FPV[0], CANData->FPV[1], CANData->FPV[2], CANData->FPV[3], CANData->FPV[4]);
            UARTprintf("highTemp: %c%c%c%c%c\n", CANData->highTemp[0], CANData->highTemp[1], CANData->highTemp[2], CANData->highTemp[3], CANData->highTemp[4]);
            UARTprintf("lowTemp: %c%c%c%c%c\n", CANData->lowTemp[0], CANData->lowTemp[1], CANData->lowTemp[2], CANData->lowTemp[3], CANData->lowTemp[4]);
            UARTprintf("highVoltage: %c%c%c%c%c\n", CANData->highVoltage[0], CANData->highVoltage[1], CANData->highVoltage[2], CANData->highVoltage[3], CANData->highVoltage[4]);
            UARTprintf("lowVoltage: %c%c%c%c%c\n", CANData->lowVoltage[0], CANData->lowVoltage[1], CANData->lowVoltage[2], CANData->lowVoltage[3], CANData->lowVoltage[4]);
            UARTprintf("RPM: %c%c%c%c%c%c\n", CANData->RPM[0], CANData->RPM[1], CANData->RPM[2], CANData->RPM[3], CANData->RPM[4], CANData->RPM[5]);
*/        }

        else
        {
            if(errFlag)
            {
                UARTprintf("Error: Problem while receiving CAN interrupt\n");
            }
        }
    }
}
