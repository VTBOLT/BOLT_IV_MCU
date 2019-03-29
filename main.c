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
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "ti/devices/msp432e4/driverlib/driverlib.h"


//*****************************************************************************
//
// The error routine that is called if the driver library encounters an error.
//
//*****************************************************************************
#ifdef DEBUG
void
__error__(char *pcFilename, uint32_t ui32Line)
{
    while(1);
}
#endif

#define FPV_length      6
#define tempLength      6
#define voltageLength   6
#define imuLength		6
#define SOC_length		6

/* Configure system clock for 120 MHz */
uint32_t systemClock;

//*****************************************************************************
//
// Flags that contain the current value of the interrupt indicator as displayed
// on the UART.
//
//*****************************************************************************

// CAN data struct
typedef struct {
    uint8_t SOC[SOC_length];
    uint8_t FPV[FPV_length];
    uint8_t highTemp[tempLength];
    uint8_t lowTemp[tempLength];
    uint8_t highVoltage[voltageLength];
    uint8_t lowVoltage[voltageLength];
} CANTransmitData;

// IMU data struct
typedef struct {
	uint8_t xLat[imuLength];
	uint8_t yLat[imuLength];
	uint8_t zLat[imuLength];
	uint8_t xGyro[imuLength];
	uint8_t yGyro[imuLength];
	uint8_t zGyro[imuLength];
} IMUTransmitData_t;

// Pump Voltage
uint8_t pumpVoltage[voltageLength];

// AUX pack voltage
uint8_t auxVoltage[voltageLength];

CANTransmitData CANData;
IMUTransmitData_t IMUData;

// Global variable to count milliseconds
uint8_t msCount = 0;

// ISR Flags
uint8_t g_ui8xbeeFlag = 0;
uint32_t g_ui32Flags;

// function prototypes
void initTimers();
void UART7Setup();
void UARTSend(const uint8_t*, uint32_t);
void initGPIO();
void xbeeTransmit(CANTransmitData, IMUTransmitData_t, uint8_t*, uint8_t*);
void UART_SendComma();
void TIMER0A_IRQHandler();
void uintToStr(char*, size_t, uint32_t);


//*****************************************************************************
//
// Blink the on-board LED.
//
//*****************************************************************************
int main(void)
{
    /* Configure system clock for 120 MHz */
    systemClock = MAP_SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ | SYSCTL_OSC_MAIN |
                                          SYSCTL_USE_PLL | SYSCTL_CFG_VCO_480),
                                          120000000);

    // Enable processor interrupts.
    MAP_IntMasterEnable();

    initTimers(systemClock);
    initGPIO();
    UART7Setup();

    memset(&CANData, '\0', sizeof(CANData));
    memset(&IMUData, '\0', sizeof(IMUData));

//    strncpy((char* )CANData.SOC, "50", sizeof(CANData.SOC));
//    strncpy((char* )CANData.FPV, "300", sizeof(CANData.FPV));
//    strncpy((char* )CANData.highTemp, "80", sizeof(CANData.highTemp));
//    strncpy((char* )CANData.lowTemp, "70", sizeof(CANData.lowTemp));
//    strncpy((char* )CANData.highVoltage, "6.8", sizeof(CANData.highVoltage));
//    strncpy((char* )CANData.lowVoltage, "6.4", sizeof(CANData.lowVoltage));
//    strncpy((char* )IMUData.xLat, "1.5", sizeof(IMUData.xLat));
//    strncpy((char* )IMUData.yLat, "1.2", sizeof(IMUData.yLat));
//    strncpy((char* )IMUData.zLat, "1", sizeof(IMUData.zLat));
//    strncpy((char* )IMUData.xGyro, "20", sizeof(IMUData.xGyro));
//    strncpy((char* )IMUData.yGyro, "3", sizeof(IMUData.yGyro));
//    strncpy((char* )IMUData.zGyro, "0", sizeof(IMUData.zGyro));
//    strncpy((char* )pumpVoltage, "12", sizeof(pumpVoltage));
//    strncpy((char* )auxVoltage, "16", sizeof(auxVoltage));

    char temp[20];
    memset(&temp, '\0', sizeof(temp));

    while(1)
    {
    	if (g_ui8xbeeFlag) {

    		//uint32_t msBefore = msCount;
        	xbeeTransmit(CANData, IMUData, pumpVoltage, auxVoltage);
        	//uint32_t msAfter = msCount;

        	//uintToStr(temp, sizeof(temp), msBefore);
        	//UARTSend((uint8_t *)temp, sizeof(temp));
        	//uintToStr(temp, sizeof(temp), msAfter);
        	//UARTSend((uint8_t *)temp, sizeof(temp));

        	g_ui8xbeeFlag = 0;
    	}
    }
}

void uintToStr(char* buf, size_t sz, uint32_t num)
{
    char* temp = (char *)malloc(sz*sizeof(char));
	char* delptr = temp;
	char* src = buf;

	do *temp++ = num % 10 + '0';
	while ((num /= 10) >= 10);
	*temp = num % 10 + '0';

    while ((*src++ = *temp--) && --sz);
    if (sz-- > 1) do *src++ = '\0'; while (--sz);
    free(delptr);
}

void initTimers(uint32_t sysClock)
{
    // Enable the peripherals used by this example.
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);

    // Configure timer 0
    MAP_TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);
    MAP_TimerLoadSet(TIMER0_BASE, TIMER_A, sysClock / 1000);

    // Setup the interrupts for the timer timeouts.
    MAP_IntEnable(INT_TIMER0A);
    MAP_TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);

    // Enable the timers.
    MAP_TimerEnable(TIMER0_BASE, TIMER_A);
}

void initGPIO()
{
    // Enable the GPIO port that is used for the on-board LED.
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION);

    // Check if the peripheral access is enabled.
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPION));

    // Enable the GPIO pin for the LED (PN0).  Set the direction as output, and
    // enable the GPIO pin for digital function.
    GPIOPinTypeGPIOOutput(GPIO_PORTN_BASE, GPIO_PIN_0);
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

void UARTSend(const uint8_t *pui8Buffer, uint32_t ui32Count)
{
    // Loop while there are more characters to send.
    while(ui32Count--)
    {
        // Write the next character to the UART.
        //MAP_UARTCharPutNonBlocking(UART7_BASE, *pui8Buffer++);
    	MAP_UARTCharPut(UART7_BASE, *pui8Buffer++);
    }
}

void xbeeTransmit(CANTransmitData CANData, IMUTransmitData_t IMUData, uint8_t* pumpVoltage, uint8_t* auxVoltage)
{
	// Send CAN Data
	UARTSend(CANData.SOC, sizeof(CANData.SOC));
	UART_SendComma();
	UARTSend(CANData.FPV, sizeof(CANData.FPV));
	UART_SendComma();
	UARTSend(CANData.highTemp, sizeof(CANData.highTemp));
	UART_SendComma();
	UARTSend(CANData.lowTemp, sizeof(CANData.lowTemp));
	UART_SendComma();
	UARTSend(CANData.highVoltage, sizeof(CANData.highVoltage));
	UART_SendComma();
	UARTSend(CANData.lowVoltage, sizeof(CANData.lowVoltage));
	UART_SendComma();

	// Send Pump Voltage
	UARTSend(pumpVoltage, sizeof(pumpVoltage));
	UART_SendComma();

	// Send AUX pack voltage
	UARTSend(auxVoltage, sizeof(auxVoltage));
	UART_SendComma();

	// Send IMU Data
	UARTSend(IMUData.xLat, sizeof(IMUData.xLat));
	UART_SendComma();
	UARTSend(IMUData.yLat, sizeof(IMUData.yLat));
	UART_SendComma();
	UARTSend(IMUData.zLat, sizeof(IMUData.zLat));
	UART_SendComma();
	UARTSend(IMUData.xGyro, sizeof(IMUData.xGyro));
	UART_SendComma();
	UARTSend(IMUData.yGyro, sizeof(IMUData.yGyro));
	UART_SendComma();
	UARTSend(IMUData.zGyro, sizeof(IMUData.zGyro));
}

void UART_SendComma()
{
	UARTSend(",", 1);
}

void TIMER0A_IRQHandler(void)
{

    // Clear the timer interrupt.
    MAP_TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);


    // toggle LED every 0.25 seconds
    if (msCount >= 250) {
        // Toggle the flag for the first timer.
        HWREGBITW(&g_ui32Flags, 0) ^= 1;
        g_ui8xbeeFlag = 1;
    	GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_0, g_ui32Flags);
    	msCount = 0;
    }
    msCount++;
}
