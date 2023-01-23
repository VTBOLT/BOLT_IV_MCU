/******************************************************************************
*
* MSP432 BOLT IV MCU PROGRAM
*
* Authors:
* William Campbell
* Ethan Brooks
* Patrick Graybeal
* Logan Richardson
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
* Must include "MAP_" before driverlib function calls, I don't know why, but
*   otherwise it doesn't actually see the function and doesn't compile.
*
* So that UART2 is connected to the XDS110 and CAN0 is enabled to RX/TX,
*   the jumpers on the MCU must be horizontal (see silkscreen graphic)
*
******************************************************************************/

#include "msp.h"

/* Standard driverlib include - can be more specific if needed */
#include <ti/devices/msp432e4/driverlib/driverlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "uartstdio.h"

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

#define auxVoltageID     0xC00
#define auxVoltageLength 4

#define motorTempID		 0x0A2
#define motorTempByte	 0
#define motorCtrlTempID  0x0A0
#define motorCtrlTempByte 6
#define motorTempLength  6 // Motor temperature ranges from -3276.8 to +3276.7 C
#define motorCtrlTempLen 6

#define dcBusCurrentID	 0x0A6
#define dcBusCurrentByte 6
#define dcBusCurrentLen	 6 // DC Bus current ranges from -3276.8 to +3276.7 Amps

#define motorTorqueID	 0x0AC
#define motorTorqueByte  0
#define motorTorqueLen	 6 // Motor torque ranges from -3276.8 to +3276.7 N-m

#define highVoltageID    0x6B3
#define highVoltageByte  2
#define lowVoltageID     0x6B3
#define lowVoltageByte   4
#define voltageLength    5
#define imuLength		 9

#define RPM_ID           0x0A5
#define RPM_BYTE         2
#define RPM_LEN          6 //due to possible negative, will have leading 0 if positive

#define UART_2_EN
//#define XBEE_PLACEHOLDER_DATA

#define IGNIT_CUTOFF_DELAY	70
#define IMU_RECEIVE_BAUD 57600
#define XBEE_BAUD_RATE 57600

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
    uint8_t RPM[RPM_LEN];                // 1 (rpm)
    uint8_t motorTemp[motorTempLength];
    uint8_t motorCtrlTemp[motorCtrlTempLen];
    uint8_t motorTorque[motorTorqueLen];
    uint8_t dcBusCurrent[dcBusCurrentLen];
} CANTransmitData_t;


// IMU data struct
typedef struct {
	uint8_t xAcc[imuLength];
	uint8_t yAcc[imuLength];
	uint8_t zAcc[imuLength];
	uint8_t xGyro[imuLength];
	uint8_t yGyro[imuLength];
	uint8_t zGyro[imuLength];
	uint8_t pitch[imuLength];
	uint8_t roll[imuLength];
	uint8_t yaw[imuLength];
	uint8_t compass[imuLength];
} IMUTransmitData_t;

// Instantiate IMU object
IMUTransmitData_t gIMUData;

// IMU receive buffer
char gIMUReceiveBuf[imuLength];
uint32_t gIMUBufIndex = 0;

// Pump Voltage
uint8_t pumpVoltage[voltageLength];

// AUX pack voltage
uint8_t auxVoltage[voltageLength];

// Global variable to count milliseconds
uint8_t msCount = 0;

// ISR Flags
uint8_t g_ui8xbeeFlag = 0;
uint32_t g_ui32Flags;

/* Temp */
uint8_t CANCount = 0;
uint8_t g_ui8canFlag = 0;

// Timer 1 flag
uint8_t intTimer1_flag = 0;

// Debounce counter
uint32_t debounceCounter = 0;

typedef enum states { PCB, ACC, IGN, MAX_STATES } states_t;
typedef enum ignitState { IGNIT_OFF = 0, IGNIT_ON = 1 } ignitState_t;
// Required second count to delay switching off ACC and IGN if voltage dips below for a brief second.
// We want to calculate the # of cycles to delay x seconds from the formula below:
// x seconds delay = (# of Cycles) * (1/frequency)
// So plug in the second count that you want delayed into the formula below:
// (120MHz)*(x second delay) = # of cycles
// The current value is for a 3 second delay: (120MHz)*(3s)=360000000
#define REQSECCOUNT 360000000

// Function prototypes
//void UARTSend(const uint8_t *pui8Buffer, uint32_t ui32Count);
void UARTSendChar(uint32_t, const char);
void UARTSendCharNonBlocking(uint32_t, const char);
void UARTSendStr(uint32_t, const uint8_t*, uint32_t);
void UARTSendStrNonBlocking(uint32_t, const uint8_t*, uint32_t);
//void UART_SendComma();
void ADCSetup();
uint32_t auxADCSend(uint32_t* auxBatVoltage);
uint32_t pumpADCSend(uint32_t* pumpVoltage);
void canSendData(int id, char* data); // send AUX battery voltage over CAN
void canSendData_Int(int id, int data, int digits);
void UART7Setup();
void UART6Setup(void);
void accIgnDESetup(void);
void timerSetup();
void timerRun();
bool ignitDebounce(bool, uint32_t*, uint8_t*);
bool ignitPoll(void);
bool accPoll(void);
bool DEPoll(void);
void canSetup(tCANMsgObject* message);
void configureCAN();
void canReceive(tCANMsgObject* sCANMessage, CANTransmitData_t* CANdata,
                uint8_t msgDataIndex, uint8_t* msgData);
// This function can handle signed and unsigned from -32767 to +32767
void convertToASCII(uint8_t* chars, uint8_t digits, int32_t num);
void enableUARTprintf();
void initTimers();
void TIMER1A_IRQHandler();
void UART6_IRQHandler(void);
void xbeeTransmit(CANTransmitData_t, IMUTransmitData_t, uint8_t*, uint8_t*);
void imuParse(char c);

int main(void)
{
    /* Configure system clock for 120 MHz */
    systemClock = MAP_SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ | SYSCTL_OSC_MAIN |
                                          SYSCTL_USE_PLL | SYSCTL_CFG_VCO_480),
                                          120000000);
    // Set up the timer system
    timerSetup();
    initTimers(systemClock);

    // Instantiate CAN objects
    static CANTransmitData_t CANData;
    tCANMsgObject sCANMessage;
    uint8_t msgDataIndex;
    uint8_t msgData[8] = {0x00, 0x00, 0x00, 0x00,
                          0x00, 0x00, 0x00, 0x00};

    // Initialize IMU object
    memset(gIMUData.xAcc, '0', imuLength);
    memset(gIMUData.yAcc, '0', imuLength);
    memset(gIMUData.zAcc, '0', imuLength);
    memset(gIMUData.xGyro, '0', imuLength);
    memset(gIMUData.yGyro, '0', imuLength);
    memset(gIMUData.zGyro, '0', imuLength);
    memset(gIMUData.roll, '0', imuLength);
    memset(gIMUData.pitch, '0', imuLength);
    memset(gIMUData.yaw, '0', imuLength);
    memset(gIMUData.compass, '0', imuLength);


    memset(auxVoltage, '0', sizeof(auxVoltage));
    uint32_t auxBatVoltage[1];
    uint32_t auxBatAdjusted; //no decimal, accurate value

    while(!(MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_ADC0)))

    ADCSetup();
    UART7Setup();
    UART6Setup();
    accIgnDESetup();
    configureCAN();
    canSetup(&sCANMessage);

    // Enable interrupts globally
    MAP_IntMasterEnable();

    enableUARTprintf();
    UARTprintf("UARTprintf enabled\n");

    states_t present = PCB;

#ifdef XBEE_PLACEHOLDER_DATA
	strncpy((char* )CANData.SOC, "50", sizeof(CANData.SOC));
	strncpy((char* )CANData.FPV, "300", sizeof(CANData.FPV));
	strncpy((char* )CANData.highTemp, "80", sizeof(CANData.highTemp));
	strncpy((char* )CANData.lowTemp, "70", sizeof(CANData.lowTemp));
	strncpy((char* )CANData.highVoltage, "6.8", sizeof(CANData.highVoltage));
	strncpy((char* )CANData.lowVoltage, "6.4", sizeof(CANData.lowVoltage));
	strncpy((char* )IMUData.xAcc, "1.5", sizeof(IMUData.xAcc));
	strncpy((char* )IMUData.yAcc, "1.2", sizeof(IMUData.yAcc));
	strncpy((char* )IMUData.zAcc, "1", sizeof(IMUData.zAcc));
	strncpy((char* )IMUData.xGyro, "20", sizeof(IMUData.xGyro));
	strncpy((char* )IMUData.yGyro, "3", sizeof(IMUData.yGyro));
	strncpy((char* )IMUData.zGyro, "0", sizeof(IMUData.zGyro));
	strncpy((char* )pumpVoltage, "12", sizeof(pumpVoltage));
	strncpy((char* )auxVoltage, "16", sizeof(auxVoltage));
#endif

	//UARTprintf("Entering loop");
    // Loop forever.
    while (1)
    {

        if (g_ui8xbeeFlag) {
    		xbeeTransmit(CANData, gIMUData, pumpVoltage, auxVoltage);
    		g_ui8xbeeFlag = 0;
    	}

        // As long as the PCB is on, CAN should be read
    	if (g_ui8canFlag) {
    	    canSendData_Int(31, auxADCSend(auxBatVoltage), 4);
    	    canReceive(&sCANMessage, &CANData, msgDataIndex, msgData);
    	    g_ui8canFlag = 0;
    	    //UARTprintf("Compare value: %i\n", auxADCSend(auxBatVoltage));
            //UARTprintf("Pump: %i", pumpADCSend(pumpVoltage));
    	}

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

            //UARTprintf("Aux Bat: %i", auxBatAdjusted);
            //UARTprintf("Pump: %i", pumpADCSend(pumpVoltage));

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

                //UARTprintf("Pump: %i", pumpADCSend(pumpVoltage));

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

//Takes in id of voltage and the current voltage and sends the voltage value over CAN
void canSendData(int id, char* data)
{
    uint32_t dataLength = strlen(data);
    uint8_t msgData[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    //UARTprintf("Length is %d \n", dataLength);
    int i;
    
    // fill the beginning of the message with null characters
    for(i=0; i<8-dataLength; i++)
    {
        msgData[i] = 0x00;
    }

    // fill the rest of the 8 bytes with the actual message
    for(i=8-dataLength; i<8; i++)
    {
        msgData[i] = (uint8_t)(data[i-8+dataLength]);
        //UARTprintf("%X", (ASCII)data[i-8+dataLength]);
    }

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

// takes in the voltage value as an integer and converts it to a string
void canSendData_Int(int id, int data, int digits){
    char str[digits];
    sprintf(str, "%d", data);
    canSendData(id, str);
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
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION);
    while(!MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_GPION));
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOP);
    while(!MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOP)){};
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOQ);
    while(!MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOQ)){};

    // Enable the GPIO pin for the LED (PN0).  Set the direction as output, and
    // enable the GPIO pin for digital function.
    MAP_GPIOPinTypeGPIOOutput(GPIO_PORTN_BASE, GPIO_PIN_0);
    MAP_GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_0, GPIO_PIN_0);


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

bool ignitDebounce(bool btn, uint32_t* count, uint8_t* flag)
{
	static ignitState_t ignitState = IGNIT_OFF;
	static uint32_t bounceCount = 0; // count number of ms that ignition switch has bounced
	static bool bounceFlag = false;

	switch (ignitState)
	{
	case IGNIT_OFF:
		if (btn == true) {
			ignitState = IGNIT_ON;
		}
		// count number of times the ignition switch is bouncing
		if (bounceFlag) {
			bounceCount++;
			bounceFlag = false;
		}
		break;

	case IGNIT_ON:
		if (*flag) {
			*flag = 0;
			if (btn == false) {
				bounceFlag = true;
				if (*count > IGNIT_CUTOFF_DELAY) {
					*count = 0;
					ignitState = IGNIT_OFF;
				}
				else {
					(*count)++;
				}
			}
			else {
				*count = 0; // reset counter if ignition switch turns back on
			}
		}
		break;
	}
	//UARTprintf("%d\n", *count);
	return ignitState;
}

bool ignitPoll(void)
{
    /* Return TRUE if ignition Rx reads HIGH and FALSE if ignition Rx reads LOW
       As a note: IGN Rx: PP3, IGN Relay: PH0 */

//    if (  MAP_GPIOPinRead(GPIO_PORTP_BASE, GPIO_PIN_3) == GPIO_PIN_3) {
//        return true;
//    } else {
//        return false;
//    }
	return ignitDebounce((bool)(MAP_GPIOPinRead(GPIO_PORTP_BASE, GPIO_PIN_3)), &debounceCounter, &intTimer1_flag);
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

void UARTSendStrNonBlocking(uint32_t UART_BASE, const uint8_t *pui8Buffer, uint32_t ui32Count)
{
    //
    // Loop while there are more characters to send.
    //
    while(ui32Count--)
    {
        //
        // Write the next character to the UART.
        //
        MAP_UARTCharPutNonBlocking(UART_BASE, *pui8Buffer++);
    }
}

void UARTSendCharNonBlocking(uint32_t UART_BASE, const char c)
{
	MAP_UARTCharPutNonBlocking(UART_BASE, c);
}

void UARTSendStr(uint32_t UART_BASE, const uint8_t *pui8Buffer, uint32_t ui32Count)
{
    //
    // Loop while there are more characters to send.
    //
    while(ui32Count--)
    {
        //
        // Write the next character to the UART.
        //
        MAP_UARTCharPut(UART_BASE, *pui8Buffer++);
    }
}

void UARTSendChar(uint32_t UART_BASE, const char c)
{
	MAP_UARTCharPut(UART_BASE, c);
}

//void UARTSend(const uint8_t *pui8Buffer, uint32_t ui32Count)
//{
//    //
//    // Loop while there are more characters to send.
//    //
//    while(ui32Count--)
//    {
//        //
//        // Write the next character to the UART.
//        //
//        MAP_UARTCharPut(UART7_BASE, *pui8Buffer++);
//    }
//}

void ADCSetup()
{
    /* AUX ADC SETUP - built using adc0_singleended_singlechannel_singleseq */

    /* Enable the clock to GPIO Ports E & D and wait for it to be ready */
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    while(!(MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOE))) {};
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    while(!(MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOD))) {};

    /* Configure PE0 as ADC input channel */
    MAP_GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_0); // aux battery
    MAP_GPIOPinTypeADC(GPIO_PORTD_BASE, GPIO_PIN_7); // pump

    /* Enable the clock to ADC0 and ADC1 and wait for it to be ready */
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
    while(!(MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_ADC0))) {};
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC1);
    while(!(MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_ADC1))) {};

    /* Configure Sequencer 3 to sample a single analog channel: AIN3 */
    MAP_ADCSequenceStepConfigure(ADC0_BASE, 3, 0, ADC_CTL_CH3 | ADC_CTL_IE | ADC_CTL_END);
    /* Configure sequencer 3 on ADC1 */
    MAP_ADCSequenceStepConfigure(ADC1_BASE, 3, 0, ADC_CTL_CH3 | ADC_CTL_IE | ADC_CTL_END);

    /* Configure and enable sample sequence 3 with a processor signal trigger */
    MAP_ADCSequenceConfigure(ADC0_BASE, 3, ADC_TRIGGER_PROCESSOR, 0);
    MAP_ADCSequenceEnable(ADC0_BASE, 3);
    MAP_ADCSequenceConfigure(ADC1_BASE, 3, ADC_TRIGGER_PROCESSOR, 0);
    MAP_ADCSequenceEnable(ADC1_BASE, 3);

    /* Clear interrupt status flag */
    MAP_ADCIntClear(ADC0_BASE, 3);
    MAP_ADCIntClear(ADC1_BASE, 3);
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

    float tempFloat = auxBatVoltage[0];

    // From ((v/1000)/1.265)/.1904 - see spreadsheet
    //tempFloat *= 0.004719;
    //tempFloat *= 0.004104;
    //uint32_t temp = tempFloat * 100;
    //uint32_t toReturn = temp;

    uint32_t tempTrueVoltage = (tempFloat / 218.587) * 100; // see spreadsheet
    uint32_t compareVoltage = tempTrueVoltage;

    //UARTprintf("Raw Value: %i\n", auxBatVoltage[0]);
    //UARTprintf("Adjusted Value: %i\n", tempTrueVoltage);

    convertToASCII(auxVoltage, voltageLength-1, tempTrueVoltage);

    // Adjust for decimal point in the middle to transmit
    auxVoltage[4] = auxVoltage[3];
    auxVoltage[3] = auxVoltage[2];
    auxVoltage[2] = '.';

    /*
    uint32_t tempToTransmit = auxBatVoltage[0] * .461;
    UARTprintf("Aux bat to transmit: %i\n", tempToTransmit);


    asciiChars[4] = tempToTransmit % 10 + '0';
    tempToTransmit /= 10;
    asciiChars[3] = tempToTransmit % 10 + '0';
    tempToTransmit /= 10;
    asciiChars[2] = '.';
    asciiChars[1] = tempToTransmit % 10 + '0';
    tempToTransmit /= 10;
    asciiChars[0] = tempToTransmit % 10 + '0';
    */

    //UARTprintf("Put into chars: %c%c%c%c%c\n", auxVoltage[0], auxVoltage[1], auxVoltage[2], auxVoltage[3], auxVoltage[4]);

    /*
    uint8_t i = 0;
    for ( ; i < 5; i++) {
        auxVoltage[i] = asciiChars[i];
    }
    */

    return compareVoltage;
}

uint32_t pumpADCSend(uint32_t* pumpVoltage)
{
    /* AUX ADC */
    MAP_ADCProcessorTrigger(ADC1_BASE, 3);
    while(!MAP_ADCIntStatus(ADC1_BASE, 3, false)) {}
    MAP_ADCIntClear(ADC1_BASE, 3);
    MAP_ADCSequenceDataGet(ADC1_BASE, 3, pumpVoltage);
    return pumpVoltage[0];
}

void UART6Setup(void)
{
    /* UART Transmit Setup */

    /* Enable clock to peripherals used */
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART6);
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOP);

    /* Set PP0 and PP1 as UART pins */
    GPIOPinConfigure(GPIO_PP0_U6RX);
    GPIOPinConfigure(GPIO_PP1_U6TX);
    MAP_GPIOPinTypeUART(GPIO_PORTP_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    /* Configure UART for 57,600, 8-N-1 */
    MAP_UARTConfigSetExpClk(UART6_BASE, systemClock, IMU_RECEIVE_BAUD,
                            UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE);

	MAP_IntEnable(INT_UART6);
	MAP_UARTIntEnable(UART6_BASE, UART_INT_RX | UART_INT_RT);
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

void initTimers(uint32_t sysClock)
{
    // Enable the peripherals used by this example.
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1);

    // Configure timer 1
    MAP_TimerConfigure(TIMER1_BASE, TIMER_CFG_PERIODIC);
    MAP_TimerLoadSet(TIMER1_BASE, TIMER_A, sysClock / 1000);

    // Setup the interrupts for the timer timeouts.
    MAP_IntEnable(INT_TIMER1A);
    MAP_TimerIntEnable(TIMER1_BASE, TIMER_TIMA_TIMEOUT);

    // Enable the timers.
    MAP_TimerEnable(TIMER1_BASE, TIMER_A);
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

    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOL);
    while(!MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOL)){};

    MAP_GPIOPinTypeGPIOOutput(GPIO_PORTL_BASE, GPIO_PIN_3);
    MAP_GPIOPinWrite(GPIO_PORTL_BASE, GPIO_PIN_3, ~(GPIO_PIN_3));

    /* Initialize the CAN controller */
    MAP_CANInit(CAN0_BASE);

    /* Set up the bit rate for the CAN bus.  CAN bus is set to 500 Kbps */
    MAP_CANBitRateSet(CAN0_BASE, systemClock, 250000);

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

void convertToASCII(uint8_t* chars, uint8_t digits, int32_t num)
{
    uint8_t STOP = 0;
    if (num < 0) {
        chars[0] = '-';
        STOP = 1;
    }
    num = (uint16_t)abs(num);

    for ( ; digits > STOP; digits--)
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
        /*
        UARTprintf("canStatus: %08X\n", canStatus);

        uint32_t rxErr, txErr;
        MAP_CANErrCntrGet(CAN0_BASE, &rxErr, &txErr);
        UARTprintf("RX Error: %08X\n", rxErr);
        UARTprintf("TX Error: %08X\n", txErr);*/
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

void xbeeTransmit(CANTransmitData_t CANData, IMUTransmitData_t IMUData, uint8_t* pumpVoltage, uint8_t* auxVoltage)
{
	// Send CAN Data
	UARTSendStr(UART7_BASE, CANData.SOC, sizeof(CANData.SOC));
	UARTSendChar(UART7_BASE, ',');
	UARTSendStr(UART7_BASE, CANData.FPV, sizeof(CANData.FPV));
	UARTSendChar(UART7_BASE, ',');
	UARTSendStr(UART7_BASE, CANData.highTemp, sizeof(CANData.highTemp));
	UARTSendChar(UART7_BASE, ',');
	UARTSendStr(UART7_BASE, CANData.lowTemp, sizeof(CANData.lowTemp));
	UARTSendChar(UART7_BASE, ',');
	UARTSendStr(UART7_BASE, CANData.highVoltage, sizeof(CANData.highVoltage));
	UARTSendChar(UART7_BASE, ',');
	UARTSendStr(UART7_BASE, CANData.lowVoltage, sizeof(CANData.lowVoltage));
	UARTSendChar(UART7_BASE, ',');
	UARTSendStr(UART7_BASE, CANData.RPM, sizeof(CANData.RPM));
	UARTSendChar(UART7_BASE, ',');
	UARTSendStr(UART7_BASE, CANData.motorTemp, sizeof(CANData.motorTemp));
	UARTSendChar(UART7_BASE, ',');
	UARTSendStr(UART7_BASE, CANData.dcBusCurrent, sizeof(CANData.dcBusCurrent));
	UARTSendChar(UART7_BASE, ',');
	UARTSendStr(UART7_BASE, CANData.motorTorque, sizeof(CANData.motorTorque));
	UARTSendChar(UART7_BASE, ',');
	UARTSendStr(UART7_BASE, CANData.motorCtrlTemp, sizeof(CANData.motorCtrlTemp));
	UARTSendChar(UART7_BASE, ',');

	// Send Pump Voltage
	//UARTSend(pumpVoltage, sizeof(pumpVoltage));
	//UART_SendComma();

	// Send AUX pack voltage
	UARTSendStr(UART7_BASE, auxVoltage, sizeof(auxVoltage));
	UARTSendChar(UART7_BASE, ',');

	// Send IMU Data
	UARTSendStr(UART7_BASE, IMUData.xAcc, sizeof(IMUData.xAcc));
	UARTSendChar(UART7_BASE, ',');
	UARTSendStr(UART7_BASE, IMUData.yAcc, sizeof(IMUData.yAcc));
	UARTSendChar(UART7_BASE, ',');
	UARTSendStr(UART7_BASE, IMUData.zAcc, sizeof(IMUData.zAcc));
	UARTSendChar(UART7_BASE, ',');
	UARTSendStr(UART7_BASE, IMUData.xGyro, sizeof(IMUData.xGyro));
	UARTSendChar(UART7_BASE, ',');
	UARTSendStr(UART7_BASE, IMUData.yGyro, sizeof(IMUData.yGyro));
	UARTSendChar(UART7_BASE, ',');
	UARTSendStr(UART7_BASE, IMUData.zGyro, sizeof(IMUData.zGyro));
	UARTSendChar(UART7_BASE, ',');
	UARTSendStr(UART7_BASE, IMUData.roll, sizeof(IMUData.roll));
	UARTSendChar(UART7_BASE, ',');
	UARTSendStr(UART7_BASE, IMUData.pitch, sizeof(IMUData.pitch));
}

void imuParse(char c)
{
#define DEFAULT 0
	enum {
		ACCEL = 'a',
		GYRO = 'g',
		EULER = 'e',
		ACCEL_X = 'a' + 'x',
		ACCEL_Y = 'a' + 'y',
		ACCEL_Z = 'a' + 'z',
		GYRO_X = 'g' + 'x',
		GYRO_Y = 'g' + 'y',
		GYRO_Z = 'g' + 'z',
		EULER_R = 'e' + 'r',
		EULER_Y = 'e' + 'y',
		EULER_P = 'e' + 'p',
		COMPASS = 'c'
	};
	static int state = DEFAULT; // initial state will be default state

	switch (state)
	{
	case ACCEL:
		state = 'a' + c;
		break;

	case ACCEL_X:
		memcpy(gIMUData.xAcc, gIMUReceiveBuf, imuLength);
		memset(gIMUReceiveBuf, '0', imuLength);
		gIMUBufIndex = 0;
		//UARTprintf(" -- Accel-X\n\r");
		state = DEFAULT;
		break;

	case ACCEL_Y:
		memcpy(gIMUData.yAcc, gIMUReceiveBuf, imuLength);
		memset(gIMUReceiveBuf, '0', imuLength);
		gIMUBufIndex = 0;
		//UARTprintf(" -- Accel-Y\n\r");
		state = DEFAULT;
		break;

	case ACCEL_Z:
		memcpy(gIMUData.zAcc, gIMUReceiveBuf, imuLength);
		memset(gIMUReceiveBuf, '0', imuLength);
		gIMUBufIndex = 0;
		//UARTprintf(" -- Accel-Z\n\r");
		state = DEFAULT;
		break;

	case GYRO:
		state = 'g' + c;
		break;

	case GYRO_X:
		memcpy(gIMUData.xGyro, gIMUReceiveBuf, imuLength);
		memset(gIMUReceiveBuf, '0', imuLength);
		gIMUBufIndex = 0;
		//UARTprintf(" -- Gyro-X\n\r");
		state = DEFAULT;
		break;

	case GYRO_Y:
		memcpy(gIMUData.yGyro, gIMUReceiveBuf, imuLength);
		memset(gIMUReceiveBuf, '0', imuLength);
		gIMUBufIndex = 0;
		//UARTprintf(" -- Gyro-Y\n\r");
		state = DEFAULT;
		break;

	case GYRO_Z:
		memcpy(gIMUData.zGyro, gIMUReceiveBuf, imuLength);
		memset(gIMUReceiveBuf, '0', imuLength);
		gIMUBufIndex = 0;
		//UARTprintf(" -- Gyro-Z\n\r");
		state = DEFAULT;
		break;

	case EULER:
		state = 'e' + c;
		break;

	case EULER_R:
		memcpy(gIMUData.roll, gIMUReceiveBuf, imuLength);
		memset(gIMUReceiveBuf, '0', imuLength);
		gIMUBufIndex = 0;
		//UARTprintf(" -- Roll\n\r");
		state = DEFAULT;
		break;

	case EULER_Y:
		memcpy(gIMUData.yaw, gIMUReceiveBuf, imuLength);
		memset(gIMUReceiveBuf, '0', imuLength);
		gIMUBufIndex = 0;
		//UARTprintf(" -- Yaw\n\r");
		state = DEFAULT;
		break;

	case EULER_P:
		memcpy(gIMUData.pitch, gIMUReceiveBuf, imuLength);
		memset(gIMUReceiveBuf, '0', imuLength);
		gIMUBufIndex = 0;
		//UARTprintf(" -- Pitch\n\r");
		state = DEFAULT;
		break;

	case COMPASS:
		memcpy(gIMUData.compass, gIMUReceiveBuf, imuLength);
		memset(gIMUReceiveBuf, '0', imuLength);
		gIMUBufIndex = 0;
		//UARTprintf(" -- Compass\n\r");
		state = DEFAULT;
		break;

	default:
		if ((c >= 0x30 && c <= 0x39) || c == '.' || c == '-') { // Check if the character is an ASCII number
			// Protect against buffer overflow
			if (gIMUBufIndex >= imuLength) {
				memset(gIMUReceiveBuf, '0', imuLength);
				//UARTprintf("Buffer overflow detected\n\r");
				gIMUBufIndex = 0;
			}
			else {
				gIMUReceiveBuf[gIMUBufIndex++] = c;
				//UARTprintf("%c", c);
			}
		}
		else {
			state = c;
		}
		break;
	}
}

//void UART_SendComma()
//{
//	UARTSend(",", 1);
//}

void TIMER1A_IRQHandler()
{
    // Clear the timer interrupt.
	intTimer1_flag = 1;
    MAP_TimerIntClear(TIMER1_BASE, TIMER_TIMA_TIMEOUT);

    // toggle LED every 0.05 seconds
    if (CANCount >= 50) {
        g_ui8canFlag = 1;
        CANCount = 0;
    }
    if (msCount >= 250) {
        // Toggle the flag for the first timer.
        HWREGBITW(&g_ui32Flags, 1) ^= 1;
        g_ui8xbeeFlag = 1;

        if(g_ui32Flags) {
        	MAP_GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_0, GPIO_PIN_0);
        }
        else {
        	MAP_GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_0, ~GPIO_PIN_0);
        }

        msCount = 0;
    }
    msCount++;
    CANCount++;
}

void UART6_IRQHandler(void)
{
	//UARTprintf("Entered UART6 ISR\n");
    uint32_t ui32Status;

    //
    // Get the interrrupt status.
    //
    ui32Status = MAP_UARTIntStatus(UART6_BASE, true);

    //
    // Clear the asserted interrupts.
    //
    MAP_UARTIntClear(UART6_BASE, ui32Status);

    //
    // Loop while there are characters in the receive FIFO.
    //
    while(MAP_UARTCharsAvail(UART6_BASE))
    {
    	char c = MAP_UARTCharGetNonBlocking(UART6_BASE);
    	//UARTprintf(c);
    	//MAP_UARTCharPutNonBlocking(UART0_BASE, c);
    	imuParse(c);
    }
}
