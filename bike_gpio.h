#ifndef BIKE_GPIO_INCLUDED
#define BIKE_GPIO_INCLUDED

//Function Prototypes
void accIgnDESetup(void);
bool ignitDebounce(bool, uint32_t*, uint8_t*);
bool ignitPoll(void);
bool accPoll(void);
bool DEPoll(void);

