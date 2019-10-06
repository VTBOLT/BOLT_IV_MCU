#include "msp.h"

/* Standard driverlib include - can be more specific if needed */
#include <globals.h>
#include <ti/devices/msp432e4/driverlib/driverlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "uartstdio.h"

#include "can_comms.h"
#include "uart_comms.h"
#include "bike_gpio.h"


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




#endif
