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
#include "bike_adc.h"

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
