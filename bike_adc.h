#ifndef BIKE_ADC_INCLUDED
#define BIKE_ADC_INCLUDED

#include <stdint.h>


// Function prototypes

void ADCSetup();
uint32_t auxADCSend(uint32_t* auxBatVoltage);
uint32_t pumpADCSend(uint32_t* pumpVoltage);

// This function can handle signed and unsigned from -32767 to +32767
void convertToASCII(uint8_t* chars, uint8_t digits, int32_t num);





#endif
