/*
 * can.c
 *
 *  Created on: Jan 30, 2023
 *      Author: Molly Shear
 *              Mason DiGiorgio
 */

#include "can.h"

/* CAN variables */
bool rxMsg = false;
bool errFlag = false;
uint32_t msgCount = 0;

// Takes in id of voltage and the current voltage and sends the voltage value
// over CAN
void CANSendData(int id, int data) {
  uint32_t dataLength = strlen(data);
  uint8_t msgData[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  // UARTprintf("Length is %d \n", dataLength);
  int i;

  msgData[6] = data >> 8;
  msgData[7] = data;

  /*for(i=0; i<8; i++)
  {
      UARTprintf("%X ", msgData[i]);
  }
  UARTprintf("\n");
  */

  // send over CAN
  tCANMsgObject message;
  message.ui32MsgID = id;
  message.ui32MsgIDMask = 0;
  message.ui32Flags = MSG_OBJ_TX_INT_ENABLE;
  message.ui32MsgLen = sizeof(msgData);
  message.pui8MsgData = msgData;

  MAP_CANMessageSet(CAN0_BASE, 1, &message, MSG_OBJ_TYPE_TX);
}

/* Initializing the CAN variables */
void CANSetup(tCANMsgObject* message) {
  /* Enable the clock to the GPIO Port J and wait for it to be ready */
  MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
  while (!(MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOA))) {
  }

  configureCAN();

  /* Initialize message object 1 to be able to receive any CAN message ID.
   * In order to receive any CAN ID, the ID and mask must both be set to 0,
   * and the ID filter enabled */
  message->ui32MsgID = 0;
  message->ui32MsgIDMask = 0;

  /* Enable interrupt on RX and Filter ID */
  message->ui32Flags = MSG_OBJ_RX_INT_ENABLE | MSG_OBJ_USE_ID_FILTER;

  /* Size of message is 8 */
  message->ui32MsgLen = 8;  // could also be sizeof(msgData)

  /* Load the message object into the CAN peripheral. Once loaded an
   * interrupt will occur any time a CAN message is received. Use message
   * object 1 for receiving messages */
  MAP_CANMessageSet(CAN0_BASE, 1, message, MSG_OBJ_TYPE_RX);
}


/* Configure the CAN and its pins PA0 and PA1 @ 500Kbps */
void configureCAN(uint32_t systemClock) {

  /* Enable the clock to the GPIO Port A and wait for it to be ready */
  MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
  while (!(MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOA))) {
  }

  /* Enable CAN0 */
  MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_CAN0);

  /* Configure GPIO Pins for CAN mode */
  MAP_GPIOPinConfigure(GPIO_PA0_CAN0RX);
  MAP_GPIOPinConfigure(GPIO_PA1_CAN0TX);
  MAP_GPIOPinTypeCAN(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

  MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOL);
  while (!MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOL)) {
  };

  MAP_GPIOPinTypeGPIOOutput(GPIO_PORTL_BASE, GPIO_PIN_3);
  MAP_GPIOPinWrite(GPIO_PORTL_BASE, GPIO_PIN_3, ~(GPIO_PIN_3));

  /* Initialize the CAN controller */
  MAP_CANInit(CAN0_BASE);

  /* Set up the bit rate for the CAN bus. CAN bus is set to 500 Kbps */
  MAP_CANBitRateSet(CAN0_BASE, systemClock, 250000);

  /* Enable interrupts on the CAN peripheral */
  MAP_CANIntEnable(CAN0_BASE, CAN_INT_MASTER | CAN_INT_ERROR | CAN_INT_STATUS);

  /* Enable the CAN interrupt */
  MAP_IntEnable(INT_CAN0);

  /* Enable the CAN for operation */
  MAP_CANEnable(CAN0_BASE);
}


/* Receives CAN messages and sets variables accordingly */
void CANReceive(tCANMsgObject* sCANMessage, CANTransmitData_t* CANData,
                uint8_t msgDataIndex, uint8_t* msgData) {
  uint8_t cycleMsgs = 0;
  /* A new message is received */
  while (cycleMsgs <= 6) {
    cycleMsgs++;
    // UARTprintf("In CANReceive loop, cycleMsgs = %d\n", cycleMsgs);
    if (rxMsg) {
      /* Re-use the same message object that was used earlier to configure
       * the CAN */
      sCANMessage->pui8MsgData = (uint8_t*)msgData;

      /* Read the message from the CAN */
      MAP_CANMessageGet(CAN0_BASE, 1, sCANMessage, 0);

      // Set rxMsg to false since data has been put into the object
      // The next message will stay in the buffer until CANMessageGet() is
      // called again
      // The interrupt functi.on sets rxMsg to true to satisfy the if statement
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
        highTempTemp = (msgData[highTempByte + 1] << 8) | msgData[highTempByte];
      }
      if (sCANMessage->ui32MsgID == lowTempID) {
        lowTempTemp = (msgData[lowTempByte + 1] << 8) | msgData[lowTempByte];
      }
      if (sCANMessage->ui32MsgID == SOC_ID) {
        SOCtemp = msgData[SOC_byte];
      }
      if (sCANMessage->ui32MsgID == FPV_ID) {
        FPVtemp = (msgData[FPV_byte + 1] << 8) | msgData[FPV_byte];
      }
      if (sCANMessage->ui32MsgID == highVoltageID) {
        highVoltTemp =
            (msgData[highVoltageByte + 1] << 8) | msgData[highVoltageByte];
      }
      if (sCANMessage->ui32MsgID == lowVoltageID) {
        lowVoltTemp =
            (msgData[lowVoltageByte + 1] << 8) | msgData[lowVoltageByte];
      }
      if (sCANMessage->ui32MsgID == RPM_ID) {
        rpmTemp = (msgData[RPM_BYTE + 1] << 8) | msgData[RPM_BYTE];
      }
      if (sCANMessage->ui32MsgID == motorTempID) {
        motorTempTemp =
            (msgData[motorTempByte + 1] << 8) | msgData[motorTempByte];
      }
      if (sCANMessage->ui32MsgID == motorCtrlTempID) {
        motorCtrlTempTemp =
            (msgData[motorCtrlTempByte + 1] << 8) | msgData[motorCtrlTempByte];
      }
      if (sCANMessage->ui32MsgID == motorTorqueID) {
        motorTorqueTemp =
            (msgData[motorTorqueByte + 1] << 8) | msgData[motorTorqueByte];
      }
      if (sCANMessage->ui32MsgID == dcBusCurrentID) {
        dcBusCurrentTemp =
            (msgData[dcBusCurrentByte + 1] << 8) | msgData[dcBusCurrentByte];
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
      convertToASCII(CANData->motorCtrlTemp, motorCtrlTempLen,
                     motorCtrlTempTemp);
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
      if (errFlag) {
        UARTprintf("Error: Problem while receiving CAN interrupt\n");
      }
    }
  }
}


// Safe To Delete
void CAN0_IRQHandler() {  // Uses CAN0, on J5
  uint32_t canStatus;

  /* Read the CAN interrupt status to find the cause of the interrupt */
  canStatus = MAP_CANIntStatus(CAN0_BASE, CAN_INT_STS_CAUSE);

  /* If the cause is a controller status interrupt, then get the status */
  if (canStatus == CAN_INT_INTID_STATUS) {
    /* Read the controller status.  This will return a field of status
     * error bits that can indicate various errors */
    canStatus = MAP_CANStatusGet(CAN0_BASE, CAN_STS_CONTROL);

    /* Set a flag to indicate some errors may have occurred */
    errFlag = true;
    /*
    UARTprintf("canStatus: %08X\n", canStatus);

    uint32_t rxErr, txErr;
    MAP_CANErrCntrGet(CAN0_BASE, &rxErr, &txErr);
    UARTprintf("RX Error: %08X\n", rxErr);
    UARTprintf("TX Error: %08X\n", txErr);*/
  }

  /* Check if the cause is message object 1, which what we are using for
   * receiving messages */
  else if (canStatus == 1) {
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
  } else {
  }
}
